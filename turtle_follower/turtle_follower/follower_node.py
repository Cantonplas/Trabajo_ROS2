import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower_node')
        
        self.explorer_pose = None  
        self.target_pose = None    
        
        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio spawn...')
        self.spawn_explorer()

        
        self.subscription_target = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.target_pose_callback,
            10)
            
        self.subscription_explorer = self.create_subscription(
            Pose,
            '/explorer/pose',
            self.explorer_pose_callback,
            10)

        self.publisher_vel = self.create_publisher(Twist, '/explorer/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop) #10hz

    def spawn_explorer(self):
        request = Spawn.Request()
        request.x = 2.0
        request.y = 2.0
        request.theta = 0.0
        request.name = 'explorer'
        future = self.spawn_client.call_async(request)

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def explorer_pose_callback(self, msg):
        self.explorer_pose = msg

    def control_loop(self):
        if self.target_pose is None or self.explorer_pose is None:
            return

        
        dist_x = self.target_pose.x - self.explorer_pose.x
        dist_y = self.target_pose.y - self.explorer_pose.y
        
        distance = math.sqrt(dist_x**2 + dist_y**2)
        
        msg = Twist()


        if distance > 0.5:
            K_linear = 1.0 # Gain for linear velocity
            msg.linear.x = K_linear * distance
            
            desired_theta = math.atan2(dist_y, dist_x)
            desired_theta = math.atan2(dist_y, dist_x)
            
            diff = desired_theta - self.explorer_pose.theta
            
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
                
            K_angular = 4.0
            msg.angular.z = K_angular * diff
            
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.publisher_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_interfaces.srv import TurtleInfo
from turtle_interfaces.action import CatchTurtle  
import math
import time

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower_node')
        
        self.declare_parameter('explorer_x', 2.0)
        self.declare_parameter('explorer_y', 2.0)
        

        self.explorer_pose = None
        self.target_pose = None
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.current_distance = 999.9  
        
        self.callback_group = ReentrantCallbackGroup() #Utilizamos multi thread
        #No necesitamos locks por la existencia de GIL de python(objetos atomicos de python) 
        # y los calculos que se ejecutan en el bucle de control también son atomicos

        #E1
        self.spawn_client = self.create_client(Spawn, 'spawn', callback_group=self.callback_group)
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio spawn...')
        self.spawn_explorer()

        #E2
        self.subscription_target = self.create_subscription(
            Pose, '/turtle1/pose', self.target_pose_callback, 10, callback_group=self.callback_group)
        self.subscription_explorer = self.create_subscription(
            Pose, '/explorer/pose', self.explorer_pose_callback, 10, callback_group=self.callback_group)
        self.publisher_vel = self.create_publisher(Twist, '/explorer/cmd_vel', 10, callback_group=self.callback_group)

        # Loop control
        self.timer = self.create_timer(0.1, self.control_loop, callback_group=self.callback_group)

        #E3
        self.server_info = self.create_service(
            TurtleInfo, 'turtle_info', self.turtle_info_callback, callback_group=self.callback_group)

        # E6
        self._action_server = ActionServer(
            self,
            CatchTurtle,
            'catch_turtle',
            self.execute_action_callback,
            callback_group=self.callback_group)


    def spawn_explorer(self):
        x_param = self.get_parameter('explorer_x').get_parameter_value().double_value
        y_param = self.get_parameter('explorer_y').get_parameter_value().double_value

        if not (0.0 <= x_param <= 11.0) or not (0.0 <= y_param <= 11.0):
            self.get_logger().error(f'Fuera de límites. Usando 2.0')
            x_param = 2.0; y_param = 2.0
            
        request = Spawn.Request()
        request.x = x_param; request.y = y_param; request.theta = 0.0; request.name = 'explorer'
        self.spawn_client.call_async(request)

    def target_pose_callback(self, msg): 
        self.target_pose = msg

    def explorer_pose_callback(self, msg): 
        self.explorer_pose = msg

    def control_loop(self):
        if self.target_pose is None or self.explorer_pose is None: return

        dx = self.target_pose.x - self.explorer_pose.x
        dy = self.target_pose.y - self.explorer_pose.y
        self.current_distance = math.sqrt(dx**2 + dy**2)
        
        msg = Twist()
        if self.current_distance > 0.5:
            #He ralentizado la velocidad para testing que la action tarde en ejecutarse :p
            # k_prop = 1.0
            k_prop = 0.7
            msg.linear.x = k_prop * self.current_distance
            desired_theta = math.atan2(dy, dx)
            diff = desired_theta - self.explorer_pose.theta

            if diff > math.pi: 
                diff -= 2 * math.pi
            elif diff < -math.pi: 
                diff += 2 * math.pi

            msg.angular.z = 4.0 * diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
        self.publisher_vel.publish(msg)

    # Callback E3
    def turtle_info_callback(self, request, response):
        if self.target_pose is None or self.explorer_pose is None: return response
        self.fill_info(response)
        return response

    # Callback E6
    def execute_action_callback(self, goal_handle):
        self.get_logger().info('Recibiendo datos...')
        
        feedback_msg = CatchTurtle.Feedback()
        

        while self.current_distance > 0.6:

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Acción cancelada por cliente.')
                return CatchTurtle.Result(success=False, message="Cancelado")

            if self.target_pose and self.explorer_pose:
                self.fill_info(feedback_msg)
                goal_handle.publish_feedback(feedback_msg) #Mandamos datos de E3 como feedback

            time.sleep(1.0)

        goal_handle.succeed()
        self.get_logger().info('Tortuga alcanzada con éxito.')
        
        result = CatchTurtle.Result()
        result.success = True
        result.message = "Tortuga alcanzada con éxito."
        return result

    def fill_info(self, data_object):
        data_object.target_x = self.target_pose.x
        data_object.target_y = self.target_pose.y
        data_object.target_theta = self.target_pose.theta
        data_object.explorer_x = self.explorer_pose.x
        data_object.explorer_y = self.explorer_pose.y
        data_object.explorer_theta = self.explorer_pose.theta
        data_object.explorer_linear_vel = self.current_linear_vel
        data_object.explorer_angular_vel = self.current_angular_vel
        data_object.distance = self.current_distance

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TurtleFollower()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
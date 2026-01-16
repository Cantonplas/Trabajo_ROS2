import rclpy
from rclpy.node import Node
from turtle_interfaces.srv import TurtleInfo

art_intro = r"""
     =======================================================
     ||                                                   ||
     ||              TURTLE FOLLOWER                      ||
     ||                                                   ||
     =======================================================
     
               (..)                       (OO) 
               (__)                       (__)
            .-'    '-.                 .-'    '-.
           /          \               /          \
          |  EXPLORER  |   >>>>      |  TARGET    |
           \          /               \          /
            '-....- '                  '-....- '
             _/  \_                     _/  \_
        """


art_cerca = r"""
                        PELIGRO!
               (..)                 (OO) 
               (__)                 (__)
            .-'    '-.           .-'    '-.
           /          \         /          \
          |  EXPLORER  |  >>   |  TARGET    |
           \          /         \          /
            '-....- '            '-....- '
             _/  \_               _/  \_
        """

art_mediano = r"""
     
               (..)                         (OO) 
               (__)                         (__)
            .-'    '-.                   .-'    '-.
           /          \                 /          \
          |  EXPLORER  |    >>>>>>>    |  TARGET    |
           \          /                 \          /
            '-....- '                    '-....- '
             _/  \_                       _/  \_
        """

art_lejano = r"""
     
               (..)                              (OO) 
               (__)                              (__)
            .-'    '-.                        .-'    '-.
           /          \                      /          \
          |  EXPLORER  |    >>>>>>>>>>      |  TARGET    |
           \          /                      \          /
            '-....- '                         '-....- '
             _/  \_                            _/  \_
        """ 

class InfoClient(Node):
    
    def __init__(self):
        super().__init__('turtle_info_client')
        
        self.client = self.create_client(TurtleInfo, 'turtle_info')

        self.switch = False
        self.print_welcome_art()
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al a turtle_info..')
            
        self.timer = self.create_timer(1.0, self.send_request)

    def send_request(self):
        req = TurtleInfo.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.distance < 1.0 and self.switch == True:
                self.get_logger().info(art_cerca)
            elif response.distance < 4.0 and self.switch == True:
                self.get_logger().info(art_mediano)
            else:
                if self.switch == False:
                    self.switch = True
                else:
                    self.get_logger().info(art_lejano)
                
            self.get_logger().info('\n' + '='*30)
            self.get_logger().info(f'Distancia: {response.distance:.2f} m')
            self.get_logger().info(f'Objetivo (Turtle1): x={response.target_x:.2f}, y={response.target_y:.2f}')
            self.get_logger().info(f'Explorer: x={response.explorer_x:.2f}, y={response.explorer_y:.2f}')
            self.get_logger().info(f'Velocidad: Lin={response.explorer_linear_vel:.2f}, Ang={response.explorer_angular_vel:.2f}')
            self.get_logger().info('='*30)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


    def print_welcome_art(self):
        self.get_logger().info('\n' + art_intro)
def main(args=None):
    rclpy.init(args=args)
    node = InfoClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
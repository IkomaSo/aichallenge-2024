#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class Emergency(Node):
      def __init__(self):
          super().__init__('emergency')
          self.publisher_ = self.create_publisher(Empty, '/control/command/trigger', 10)
          while rclpy.ok():
              input("Press Enter to STOP the cart...")
              self.publisher_.publish(Empty())
              self.get_logger().info('Emergency stop')
              input("Press Enter to START the cart...")
              self.publisher_.publish(Empty())
              self.get_logger().info('Emergency stop released')
              
def main(args=None):
    rclpy.init(args=args)
    emergency = Emergency()
    rclpy.spin(emergency)
    emergency.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from autoware_auto_control_msgs.msg import AckermannControlCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class LongitudeTest(Node):
  def __init__(self):
    super().__init__('longitude_test')
    self.get_logger().info('LongitudeTest node started')
    self.command_pub = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10)
    self.control_req_pub = self.create_publisher(Bool, '/control/control_mode_request_topic', 10)
    self.odom_sub = self.create_subscription(Odometry, 'z', self.odom_callback, 10)
    self.timer = self.create_timer(0.1, self.timer_callback)
    self.odom = Odometry()
    self.control_req_pub.publish(Bool(data=True))
    
  def odom_callback(self, msg):
    self.odom = msg
    
  def timer_callback(self):
    if self.odom is  None:
      self.get_logger().info('Waiting for odometry message')
      return
    K_p = 0.5
    tar_speed = 10.0
    msg = AckermannControlCommand()
    msg.longitudinal.speed = 0.0
    msg.longitudinal.acceleration = K_p * (tar_speed - self.odom.twist.twist.linear.x)
    if msg.longitudinal.acceleration > 3.2:
      msg.longitudinal.acceleration = 3.2
    msg.longitudinal.jerk = 1.0
    msg.lateral.steering_tire_angle = 0.0
    msg.lateral.steering_tire_rotation_rate = 0.0
    self.command_pub.publish(msg)
    self.get_logger().info('Published control command: speed = %f, acceleration = %f' % (msg.longitudinal.speed, msg.longitudinal.acceleration))


def main():
  rclpy.init()
  node = LongitudeTest()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
    main()

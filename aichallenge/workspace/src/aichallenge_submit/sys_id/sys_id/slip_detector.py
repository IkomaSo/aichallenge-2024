import rclpy
from rclpy.node import Node
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from math import atan2, tan

class SlipDetector(Node):
  def __init__(self):
    super().__init__('slip_detector')
    self.get_logger().info('SlipDetector node started')
    
    self.wheel_base = self.declare_parameter('wheel_base', 1.28).value
    
    self.velocity_report_sub = self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.velocity_report_callback, 10)
    self.imu_sub = self.create_subscription(Imu, '/sensing/imu/imu_data', self.imu_callback, 10)
    self.imu_data = None
    self.steer_report_sub = self.create_subscription(SteeringReport, '/vehicle/status/steering_status', self.steer_report_callback, 10)
    self.steer_report = None
    self.wheel_base_est_buf = []

    self.heading_angle_pub = self.create_publisher(Float32, '/slip_detector/heading_angle', 10)
    self.slip_angle_pub = self.create_publisher(Float32, '/slip_detector/slip_angle', 10)
    self.wheel_base_pub = self.create_publisher(Float32, '/slip_detector/wheel_base', 10)
    
  def steer_report_callback(self, msg):
    self.steer_report = msg
    
  def imu_callback(self, msg):
    self.imu_data = msg
    
  def velocity_report_callback(self, msg):
    vel = msg.longitudinal_velocity
    if vel <= 0.0 or self.steer_report is None or self.imu_data is None:
      return
    omega = msg.heading_rate
    # omega = self.imu_data.angular_velocity.z
    heading_angle = atan2(omega * self.wheel_base, vel)
    
    wheel_base_est = tan(self.steer_report.steering_tire_angle) * vel / omega
    self.wheel_base_est_buf.append(wheel_base_est)
    if len(self.wheel_base_est_buf) > 100:
      self.wheel_base_est_buf.pop(0)
    wheel_base_est = sum(self.wheel_base_est_buf) / len(self.wheel_base_est_buf)
    self.wheel_base_pub.publish(Float32(data=wheel_base_est))

    slip_angle = self.steer_report.steering_tire_angle - heading_angle
    
    self.heading_angle_pub.publish(Float32(data=heading_angle))
    self.slip_angle_pub.publish(Float32(data=slip_angle))
    
def main():
  rclpy.init()
  node = SlipDetector()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
    
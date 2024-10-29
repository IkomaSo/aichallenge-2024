import rclpy
from rclpy.node import Node

from sklearn.neighbors import NearestNeighbors
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from tf_transformations import quaternion_from_euler

from nav_msgs.msg import Odometry

import numpy as np

class ClothoidTrajPublisher(Node):

  def __init__(self):
    super().__init__('clothoid_traj_publisher')
    self.declare_parameter('traj_file', '/aichallenge/workspace/src/aichallenge_submit/sanae_planner/trajectory/traj.csv')
    self.declare_parameter('wheel_base', 1.087)
    self.declare_parameter('speed_km_h', 20.0)
    self.declare_parameter('max_centripetal_acc', 4.0)
        
    self.traj_pub = self.create_publisher(Trajectory, '/planning/scenario_planning/trajectory', 10)
    self.odom_sub = self.create_subscription(Odometry, 'input/odom', self.odom_callback, 10)

    self.traj = np.loadtxt(self.get_parameter('traj_file').value, delimiter=',')
    self.x = self.traj[:, 0]
    self.y = self.traj[:, 1]
    self.yaw = self.traj[:, 2]
    self.curvature = self.traj[:, 3]
    
    self.wheel_base = self.get_parameter('wheel_base').value
    self.speed_mps = self.get_parameter('speed_km_h').value / 3.6
    
    self.nn = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array([self.x, self.y]).T)
    self.nearest_idx = 0

    self.odom = None
    
    while self.odom is None:
      self.get_logger().info('Waiting for odometry...')
      rclpy.spin_once(self)
      
    self.create_timer(0.1, self.publish_traj)
      
      
  def odom_callback(self, msg):
    self.odom = msg
    _, idx = self.nn.kneighbors(np.array([[self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]]))
    self.nearest_idx = idx[0][0]
    
  def publish_traj(self):
    self.speed_mps = self.get_parameter('speed_km_h').value / 3.6
    traj_msg = Trajectory()
    traj_msg.header.stamp = self.get_clock().now().to_msg()
    traj_msg.header.frame_id = 'map'
    
    for i in range(0, len(self.x)):
      traj_point = TrajectoryPoint()
      idx = (i + self.nearest_idx) % len(self.x)
      traj_point.pose.position.x = self.x[i]
      traj_point.pose.position.y = self.y[i]
      traj_point.pose.position.z = 0.0
      q = quaternion_from_euler(0, 0, self.yaw[i])
      traj_point.pose.orientation.x = q[0]
      traj_point.pose.orientation.y = q[1]
      traj_point.pose.orientation.z = q[2]
      traj_point.pose.orientation.w = q[3]
      traj_point.longitudinal_velocity_mps = self.speed_mps
      max_speed = np.sqrt(self.get_parameter('max_centripetal_acc').value / (np.abs(self.curvature[i])+1e-6))
      if self.speed_mps > max_speed:
        traj_point.longitudinal_velocity_mps = max_speed
      traj_point.front_wheel_angle_rad = np.arctan(self.curvature[i] * self.wheel_base)
      traj_msg.points.append(traj_point)
      
    self.traj_pub.publish(traj_msg)
    
def main(args=None):
  rclpy.init(args=args)
  node = ClothoidTrajPublisher()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
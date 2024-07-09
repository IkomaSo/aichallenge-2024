from sanae_planner.center_line_map import CenterLineMap

import rclpy
from rclpy.node import Node

import numpy as np
import casadi
import matplotlib.pyplot as plt
import time

from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sklearn.neighbors import NearestNeighbors 
from tf_transformations import quaternion_from_euler

class CasADiOptimizer:
  def __init__(self, center_line_map, n_points):
    # コース１周をn_pointsで分割したmap
    self.center_line_map = center_line_map
    self.n_points = n_points
    
  # st_idxから一周分の経路を最適化する
  def optimize(self, st_idx, deviations, obstacles):
    self.opti = casadi.Opti()
    self.X = self.opti.variable(1, self.n_points)
    self.opti.set_initial(self.X, np.zeros((1, self.n_points)))
    self.L, self.Dot = self.get_traj_length()
    self.Obst = self.get_obstacle_constraints(obstacles)
    
    # 初期条件
    self.init_len = 10
    self.st_idx = st_idx
    ignore_idx = []
    for i in range(self.init_len):
      idx = (st_idx + i) % self.n_points
      self.opti.subject_to(self.X[0, idx] == deviations[idx])
      ignore_idx.append(idx)
      
    self.set_cource_subject(ignore_idx)
    
    p_opts={}
    s_opts = {'print_level':0}
    # s_opts = {'print_level':0,
    #           'max_iter':100}
    self.opti.solver('ipopt', p_opts, s_opts)
    
    self.opti.minimize(1.*self.L - 50.*self.Dot + 2000.*self.Obst)
    # self.opti.callback(lambda i: self.plot_traj(self.opti.debug.value(self.X)))
    self.opti.callback(lambda i: print(f'iter: {i} L: {self.opti.debug.value(self.L)} Dot: {self.opti.debug.value(self.Dot)} Obst: {self.opti.debug.value(self.Obst)}'))
    sol = self.opti.solve()
    # self.plot_traj(sol.value(self.X))
    return sol.value(self.X)
    
    
  def set_cource_subject(self, ignore_idx, margin=2.2):
    for i in range(self.n_points):
      if i in ignore_idx:
        continue
      rb = -self.center_line_map.nearest_rb_dist[i]+margin
      lb = self.center_line_map.nearest_lb_dist[i]-margin
      min_width = .5
      if lb-rb < min_width:
        cent = (rb + lb) / 2
        rb = cent + -min_width/2
        lb = cent + min_width/2
      self.opti.subject_to(self.opti.bounded(rb, self.X[0, i], lb))
      
  def get_traj_length(self):
    l = 0
    dot_sum = 0
    for i in range(self.n_points):
      p0_idx = (i - 2 + self.n_points) % self.n_points
      _, _, normal_vec = self.center_line_map.get_boundaries(p0_idx)
      p0_x = self.center_line_map.eq_cl_x[p0_idx] + normal_vec[0] * self.X[0,p0_idx]
      p0_y = self.center_line_map.eq_cl_y[p0_idx] + normal_vec[1] * self.X[0,p0_idx]
      p1_idx = (i - 1 + self.n_points) % self.n_points
      _, _, normal_vec = self.center_line_map.get_boundaries(p1_idx)
      p1_x = self.center_line_map.eq_cl_x[p1_idx] + normal_vec[0] * self.X[0,p1_idx]
      p1_y = self.center_line_map.eq_cl_y[p1_idx] + normal_vec[1] * self.X[0,p1_idx]
      _, _, normal_vec = self.center_line_map.get_boundaries(i)
      p2_x = self.center_line_map.eq_cl_x[i] + normal_vec[0] * self.X[0,i]
      p2_y = self.center_line_map.eq_cl_y[i] + normal_vec[1] * self.X[0,i]
      diff_x_prev = p1_x - p0_x
      diff_y_prev = p1_y - p0_y
      dl_prev = (diff_x_prev**2 + diff_y_prev**2)**0.5
      prev_vec = [diff_x_prev / dl_prev, diff_y_prev / dl_prev]
      diff_x = p2_x - p1_x
      diff_y = p2_y - p1_y
      dl = (diff_x**2 + diff_y**2)**0.5
      vec = [diff_x / dl, diff_y / dl]
      
      dot = prev_vec[0] * vec[0] + prev_vec[1] * vec[1]
      ang = casadi.acos(dot)
      curve = ang / dl
      # self.opti.subject_to(curve < 100)
      # self.opti.subject_to(self.opti.bounded(0.9645, dot, 1))

      l += dl
      dot_sum += dot
    return l, dot_sum
  
  def get_obstacle_constraints(self, obstacles):
    obst = 0
    for obs in obstacles:
      for i in range(self.n_points):
        next_idx = (i + 1) % self.n_points
        _, _, normal_vec = self.center_line_map.get_boundaries(i)
        p0_x = self.center_line_map.eq_cl_x[i] + normal_vec[0] * self.X[0,i]
        p0_y = self.center_line_map.eq_cl_y[i] + normal_vec[1] * self.X[0,i]
        _, _, normal_vec = self.center_line_map.get_boundaries(next_idx)
        p1_x = self.center_line_map.eq_cl_x[next_idx] + normal_vec[0] * self.X[0,next_idx]
        p1_y = self.center_line_map.eq_cl_y[next_idx] + normal_vec[1] * self.X[0,next_idx]
        for j in range(10):
          p_x = p0_x + (p1_x - p0_x) * j / 10
          p_y = p0_y + (p1_y - p0_y) * j / 10
          dist = ((p_x - obs[0])**2 + (p_y - obs[1])**2)**0.5
          if dist < 5.0
            obst += casadi.exp(-(dist / obs[2])**2)
          # print(dist / (obs[2]+1.0))
          # obst += casadi.exp(0)
    return obst

  def plot_traj(self, vars):
    plt.cla()
    self.center_line_map.plot_map(False)
    for i in range(self.n_points):
      _, _, normal_vec = self.center_line_map.get_boundaries(i)
      p_x = self.center_line_map.eq_cl_x[i] + normal_vec[0] * vars[i]
      p_y = self.center_line_map.eq_cl_y[i] + normal_vec[1] * vars[i]
      plt.scatter(p_x, p_y, color='red', s=2)
    # plt.pause(1)
    plt.show()
    

class TrajectoryOptimizer(Node):
  def __init__(self):
    super().__init__('trajectory_optimizer')
    self.declare_parameter('lanelet2_map_path', 
              '/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm')
    self.declare_parameter('traj_points', 500)
    self.declare_parameter('opti_points', 200)
    self.declare_parameter('opti_ahead', 40)
    self.declare_parameter('wheel_base', 1.087)
    
    self.traj_pub = self.create_publisher(Trajectory, '/planning/scenario_planning/trajectory', 10)
    self.odom_sub = self.create_subscription(Odometry, 'input/odom', self.odom_callback, 10)
    self.obst_sub = self.create_subscription(Float64MultiArray, "/aichallenge/objects", self.obst_callback, 1)
    
    self.map_path = self.get_parameter('lanelet2_map_path').value
    self.traj_points = self.get_parameter('traj_points').value
    self.opti_points = self.get_parameter('opti_points').value
    self.opti_ahead = self.get_parameter('opti_ahead').value
    self.wheel_base = self.get_parameter('wheel_base').value
    self.center_line_map = CenterLineMap(self.map_path, self.opti_points)
    self.cl_nn = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array([self.center_line_map.eq_cl_x, self.center_line_map.eq_cl_y]).T)
    self.optimizer = CasADiOptimizer(self.center_line_map, self.opti_points)
    self.odom = None
    self.obstacles = []
    self.deviations = np.zeros(self.opti_points)
    self.current_idx = 0
    while self.odom is None:
      self.get_logger().info('Waiting for odometry message...')
      rclpy.spin_once(self)
    self.optimize_trajectory()
    self.create_timer(5, self.optimize_trajectory)
  
  def optimize_trajectory(self):
    self.get_logger().info('Optimizing trajectory...')
    st_idx = (self.current_idx + self.opti_ahead) % self.opti_points
    sol = None
    try:
      st = time.time()
      sol = self.optimizer.optimize(st_idx, self.deviations, self.obstacles)
      print(f'Optimization time: {time.time() - st}')
    except Exception as e:
      self.get_logger().error(f'Optimization failed: {e}')
      return
    self.get_logger().info('Optimization succeeded')
    devi_new = self.deviations.copy()
    # 2/3周分の経路を更新
    for i in range(self.opti_points*9//10):
      idx = (st_idx + i) % self.opti_points
      devi_new[idx] = sol[idx]
    self.deviations = devi_new
    

  def odom_callback(self, msg):
    self.get_logger().info('Received odometry message')
    self.odom = msg
    _, idx = self.cl_nn.kneighbors(np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y]]))
    self.current_idx = idx[0][0]
    self.publish_trajectory()
    
  def obst_callback(self, msg):
    print(msg.data)
    self.obstacles = []
    for i in range(0, len(msg.data), 4):
      x = msg.data[i]
      y = msg.data[i + 1]
      r = msg.data[i + 3]
      self.obstacles.append((x, y, r))
    print(self.obstacles)
  
  def publish_trajectory(self):
    traj = Trajectory()
    traj.header.stamp = self.odom.header.stamp
    traj.header.frame_id = 'map'
    traf_x = []
    traf_y = []
    for i in range(self.opti_points+1):
      idx = (self.current_idx + i) % self.opti_points
      normal_vec = self.center_line_map.get_normal_vector(idx)
      traf_x.append(self.center_line_map.eq_cl_x[idx] + normal_vec[0] * self.deviations[idx])
      traf_y.append(self.center_line_map.eq_cl_y[idx] + normal_vec[1] * self.deviations[idx])
    hires_x, hires_y = self.center_line_map.equidistant_interpolation(traf_x, traf_y, self.traj_points)
    for i in range(self.traj_points + 1):
      traj_point = TrajectoryPoint()
      idx = (self.current_idx + i) % self.traj_points
      prev_idx = (idx - 1 + self.traj_points) % self.traj_points
      next_idx = (idx + 1 + self.traj_points) % self.traj_points
      p_p = np.array([hires_x[prev_idx], hires_y[prev_idx]])
      p_c = np.array([hires_x[idx], hires_y[idx]])
      p_n = np.array([hires_x[next_idx], hires_y[next_idx]])
      prev_heading_vec = (p_c - p_p) / np.linalg.norm(p_c - p_p)
      heading_vec = (p_n - p_c) / np.linalg.norm(p_n - p_c)
      traj_point.pose.position.x = hires_x[idx]
      traj_point.pose.position.y = hires_y[idx]
      traj_point.pose.position.z = .0
      q = quaternion_from_euler(0, 0, np.arctan2(heading_vec[1], heading_vec[0]))
      traj_point.pose.orientation.x = q[0]
      traj_point.pose.orientation.y = q[1]
      traj_point.pose.orientation.z = q[2]
      traj_point.pose.orientation.w = q[3]
      traj_point.longitudinal_velocity_mps = 15.
      curvature = np.cross(prev_heading_vec, heading_vec) / np.linalg.norm(p_n - p_p)
      traj_point.front_wheel_angle_rad = np.arctan(self.wheel_base * curvature)
      traj.points.append(traj_point)
    self.traj_pub.publish(traj)
    


def main():
  rclpy.init()
  traj = TrajectoryOptimizer()
  rclpy.spin(traj)
  rclpy.shutdown()

if __name__ == '__main__':
  main()

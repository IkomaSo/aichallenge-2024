import numpy as np
import xml.etree.ElementTree as ET
import os
import matplotlib.pyplot as plt
from scipy import interpolate
from sklearn.neighbors import NearestNeighbors 

class CenterLineMap:
  def __init__(self, lanelet2_path, resolution):
    if not os.path.exists(lanelet2_path):
      raise FileNotFoundError(f'{lanelet2_path} does not exist.')
    self.load_map(lanelet2_path)
    self.resolution = resolution
    
    self.eq_cl_x, self.eq_cl_y = self.equidistant_interpolation(
      [x for x, y in self.lanelet_centerlines], 
      [y for x, y in self.lanelet_centerlines], 
      resolution)
    
    self.eq_lb_x, self.eq_lb_y = self.equidistant_interpolation(
      [x for x, y in self.lanelet_left_boundaries],
      [y for x, y in self.lanelet_left_boundaries],
      500)
    
    self.eq_rb_x, self.eq_rb_y = self.equidistant_interpolation(
      [x for x, y in self.lanelet_right_boundaries],
      [y for x, y in self.lanelet_right_boundaries],
      500)
    
    self.eq_cl_x_fine, self.eq_cl_y_fine = self.equidistant_interpolation(
      [x for x, y in self.lanelet_centerlines],
      [y for x, y in self.lanelet_centerlines],
      500)
    
    nn_lb = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array([self.eq_lb_x, self.eq_lb_y]).T)
    nn_rb = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array([self.eq_rb_x, self.eq_rb_y]).T)
    
    self.nearest_lb_dist, _ = nn_lb.kneighbors(np.array([self.eq_cl_x, self.eq_cl_y]).T)
    self.nearest_rb_dist, _ = nn_rb.kneighbors(np.array([self.eq_cl_x, self.eq_cl_y]).T)
    
    self.nearest_lb_dist = self.nearest_lb_dist.T[0]
    self.nearest_rb_dist = self.nearest_rb_dist.T[0]
    
    # self.plot_map()
    
  def load_map(self, lanelet2_path):
    tree = ET.parse(lanelet2_path)
    osm = tree.getroot()
    lanelet_nodes = {} # id: x, y
    lanelet_ways = {}  # id: nodes
    for node in osm.findall('node'):
      for tag in node:
        if tag.attrib['k'] == 'local_x':
          x = float(tag.attrib['v'])
        elif tag.attrib['k'] == 'local_y':
          y = float(tag.attrib['v'])
      lanelet_nodes[int(node.attrib['id'])] = (x, y)
      
    for way in osm.findall('way'):
      nodes = []
      for nd in way.findall('nd'):
        nodes.append(int(nd.attrib['ref']))
      lanelet_ways[int(way.attrib['id'])] = nodes
      
    self.lanelet_centerlines = []
    self.lanelet_left_boundaries = []
    self.lanelet_right_boundaries = []
    
    for relation in osm.findall('relation'):
      for member in relation.findall('member'):
        if member.attrib['role'] == 'centerline':
          for node in lanelet_ways[int(member.attrib['ref'])]:
            x, y = lanelet_nodes[node]
            self.lanelet_centerlines.append((x, y))
        elif member.attrib['role'] == 'left':
          for node in lanelet_ways[int(member.attrib['ref'])]:
            x, y = lanelet_nodes[node]
            self.lanelet_left_boundaries.append((x, y))
        elif member.attrib['role'] == 'right':
          for node in lanelet_ways[int(member.attrib['ref'])]:
            x, y = lanelet_nodes[node]
            self.lanelet_right_boundaries.append((x, y))
            
  def plot_map(self, show = True):
    plt.plot(self.eq_cl_x_fine, self.eq_cl_y_fine, linewidth=0.5)
    plt.scatter(self.eq_cl_x, self.eq_cl_y, s=1)
    plt.plot(self.eq_lb_x, self.eq_lb_y, linewidth=0.5)
    # plt.scatter(self.eq_lb_x, self.eq_lb_y, s=1)
    plt.plot(self.eq_rb_x, self.eq_rb_y, linewidth=0.5)
    # plt.scatter(self.eq_rb_x, self.eq_rb_y, s=1)
    
    for idx in range(len(self.eq_cl_x)):
      p_lb, p_rb, _ = self.get_boundaries(idx)
      plt.plot([p_lb[0], p_rb[0]], [p_lb[1], p_rb[1]], color='red', linewidth=0.5)
    
    plt.axis('equal')
    if show:
      plt.show()
            
  def equidistant_interpolation(self, x, y, resolution):
    t = np.linspace(0, 1., len(x))
    method = 'quadratic'
    interp_x = interpolate.interp1d(t, x, kind=method)
    interp_y = interpolate.interp1d(t, y, kind=method)
    # 細かい分割を考える
    s_fine = np.linspace(0, 1, resolution * 100)
    x_fine = interp_x(s_fine)
    y_fine = interp_y(s_fine)
    l_fine = 0
    for i in range(1, len(x_fine)):
      l_fine += np.sqrt((x_fine[i] - x_fine[i-1])**2 + (y_fine[i] - y_fine[i-1])**2)
    
    t_eq = np.linspace(0, 1, resolution)
    l_sum = 0
    t_idx = 1
    for i in range(1, len(s_fine)):
      l_sum += np.sqrt((x_fine[i] - x_fine[i-1])**2 + (y_fine[i] - y_fine[i-1])**2)
      if l_sum > t_idx * l_fine / resolution and t_idx < resolution:
        t_eq[t_idx] = s_fine[i]
        t_idx += 1
    x_eq = interp_x(t_eq)
    y_eq = interp_y(t_eq)
    return x_eq, y_eq
  
  def get_heading_vector(self, idx):
    fine_idx = len(self.eq_cl_x_fine) * idx // len(self.eq_cl_x)
    next_idx = fine_idx + 1 if fine_idx + 1 < len(self.eq_cl_x_fine) else 0
    p_c = np.array([self.eq_cl_x_fine[fine_idx], self.eq_cl_y_fine[fine_idx]])
    p_n = np.array([self.eq_cl_x_fine[next_idx], self.eq_cl_y_fine[next_idx]])
    line_vec = p_n - p_c
    line_vec = line_vec / np.linalg.norm(line_vec)
    return line_vec
  
  def get_normal_vector(self, idx):
    line_vec = self.get_heading_vector(idx)
    return np.array([-line_vec[1], line_vec[0]])
  
  def get_boundaries(self, idx):
    normal_vec = self.get_normal_vector(idx)
    p_c = np.array([self.eq_cl_x[idx], self.eq_cl_y[idx]])
    p_lb = p_c + normal_vec * self.nearest_lb_dist[idx]
    p_rb = p_c - normal_vec * self.nearest_rb_dist[idx]
    return p_lb, p_rb, normal_vec
  
  def get_nearest_idx(self, x, y):
    dist = (self.eq_cl_x - x)**2 + (self.eq_cl_y - y)**2
    return np.argmin(dist)
    
if __name__ == '__main__':
  center_line_map = CenterLineMap('/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm', 100)

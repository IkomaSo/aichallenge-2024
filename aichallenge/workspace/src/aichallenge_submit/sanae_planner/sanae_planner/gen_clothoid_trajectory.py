from center_line_map import CenterLineMap
import matplotlib.pyplot as plt
from pyclothoids import Clothoid, SolveG2
import math
import numpy as np

fig, ax = plt.subplots()
map = CenterLineMap('./custom_lanelet2_map.osm', 100)
points = []
picking_idx = None

onclick_cid = None
onpick_cid = None
onmotion_cid = None
onrelease_cid = None

is_open = True

def onclick(event):
  x = event.xdata
  y = event.ydata
  dist = 100000000.0
  for p in points:
    d = (p[0] - x)**2 + (p[1] - y)**2
    if d < dist:
      dist = d
    if dist < 1.0:
      if event.dblclick:
        points.remove(p)
        redraw()
      return
  points.append([x, y])
  redraw()
  
def onpick(event):
  global picking_idx
  picking_idx = event.ind[0]
  
def onmotion(event):
  global picking_idx
  if picking_idx is not None:
    x = event.xdata
    y = event.ydata
    points[picking_idx] = [x, y]
    redraw()
    
def onrelease(event):
  global picking_idx
  picking_idx = None
  
def onkey(event):
  global onclick_cid, onpick_cid, onmotion_cid, onrelease_cid
  if event.key == 'm':
    if onclick_cid is not None:
      fig.canvas.mpl_disconnect(onclick_cid)
      print('disconnected onclick')
    onpick_cid = fig.canvas.mpl_connect('pick_event', onpick)
    onmotion_cid = fig.canvas.mpl_connect('motion_notify_event', onmotion)
    onrelease_cid = fig.canvas.mpl_connect('button_release_event', onrelease)
    
  if event.key == 'a':
    if onpick_cid is not None:
      fig.canvas.mpl_disconnect(onpick_cid)
      fig.canvas.mpl_disconnect(onmotion_cid)
      fig.canvas.mpl_disconnect(onrelease_cid)
      print('disconnected onpick')
    onclick_cid = fig.canvas.mpl_connect('button_press_event', onclick)   
    
  if event.key == 'c':
    global is_open
    is_open = not is_open
    redraw() 
    
  if event.key == 'e':
    if len(points) < 4 or len(points) % 2 == 1:
      print('cannot generate clothoid')
    x = np.array([])
    y = np.array([])
    theta = np.array([])
    curvature = np.array([])
    ds = 0.1
    for i in range(0, len(points), 2):
      p1 = np.array(points[i])
      p2 = np.array(points[i+1])
      line_len = np.linalg.norm(p2 - p1)
      for s in np.arange(0, line_len, ds):
        x = np.append(x, p1[0] + (p2[0] - p1[0]) * s / line_len)
        y = np.append(y, p1[1] + (p2[1] - p1[1]) * s / line_len)
        theta = np.append(theta, math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
        curvature = np.append(curvature, 0)
      p3 = None
      p4 = None
      if i + 3 < len(points):
        p3 = np.array(points[i+2])
        p4 = np.array(points[i+3])
      else:
        p3 = np.array(points[0])
        p4 = np.array(points[1])
      x0, y0 = p2
      t0 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
      x1, y1 = p3
      t1 = math.atan2(p4[1] - p3[1], p4[0] - p3[0])
      clothoid_lis = SolveG2(x0, y0, t0, 0., x1, y1, t1, 0.)
      for c in clothoid_lis:
        for s in np.arange(0, c.length, ds):
          x = np.append(x, c.X(s))
          y = np.append(y, c.Y(s))
          theta = np.append(theta, c.Theta(s))
          curvature = np.append(curvature, c.ThetaD(s))
      print(x.shape, y.shape, theta.shape, curvature.shape)
      np.savetxt('traj.csv', np.array([x, y, theta, curvature]).T, delimiter=',')
    
def redraw():
  plt.cla()
  plt.plot(map.eq_lb_x, map.eq_lb_y, linewidth=1, color='black')
  plt.plot(map.eq_rb_x, map.eq_rb_y, linewidth=1, color='black')
  plt.scatter([x for x, y in points], [y for x, y in points], picker=15)
  plt.axis('equal')
  
  if len(points) > 1:
    global is_open
    for i in range(0, len(points), 2):
      if i + 1 >= len(points):
        break
      p1 = points[i]
      p2 = points[i+1]
      plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='red')
      if i + 3 < len(points) or not is_open:
        p3 = None
        p4 = None
        if i + 3 < len(points):
          p3 = points[i+2]
          p4 = points[i+3]
        else:
          p3 = points[0]
          p4 = points[1]
        x0, y0 = p2
        t0 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        x1, y1 = p3
        t1 = math.atan2(p4[1] - p3[1], p4[0] - p3[0])
        clothoid_lis = SolveG2(x0, y0, t0, 0., x1, y1, t1, 0.)
        max_thdd = 0
        for i in clothoid_lis:
          plt.plot(*i.SampleXY(500))
          if max_thdd < abs(i.ThetaDD(0)):
            max_thdd = abs(i.ThetaDD(0))
        plt.text(p2[0], p2[1], f'{max_thdd:.3f}', fontsize=15)
  plt.draw()

def main():
  global onclick_cid, onpick_cid
  onclick_cid = fig.canvas.mpl_connect('button_press_event', onclick)
  fig.canvas.mpl_connect('key_press_event', onkey)
  plt.plot(map.eq_lb_x, map.eq_lb_y, linewidth=1, color='black')
  plt.plot(map.eq_rb_x, map.eq_rb_y, linewidth=1, color='black')
  plt.scatter([x for x, y in points], [y for x, y in points])
  plt.axis('equal')
  plt.show()
  
if __name__ == '__main__':
  main()
  
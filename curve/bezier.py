import numpy as np
import matplotlib.pyplot as plt

import Box2D

# from define import *

class Bezier_traj():
  def __init__(self, world):
    self.para_len = 4
    self.param = []
    self.traj = []
    self.point_bodies = []
    self.world = world
    self.ctrl_start = Box2D.b2Vec2(0,0)

  def clear_bodies(self, bodies):
    while len(bodies) > 0:
      self.world.DestroyBody(bodies[-1])
      del(bodies[-1])
  
  def clean(self):
    self.param = []
    self.traj = []
    self.clear_bodies(self.point_bodies)

  def place_ctrl_param(self, x, y):
    if len(self.param) >= 4:
      # too many control points, starts over.
      self.clean()

    if len(self.param)==0:
      # no control points, use the gripper position as the first control point
      self.param.append([self.ctrl_start[0], self.ctrl_start[1]])

    if len(self.param) <= 4:
      # need four control points in total
      self.param.append([x/SCALE,y/SCALE])
      self.point_bodies.append(self.world.CreateStaticBody(
        position=(x/SCALE, y/SCALE),
        angle = 0.0,
        fixtures = BP_FD,
        )
      )
      self.point_bodies[-1].userData = 'control point'
      self.point_bodies[-1].color = (120/255, 0/255, 83/255)
    
    if len(self.param) ==4:
      # just enough control points, generate a trajectory
      self.traj_gen()

  def traj_gen(self, gran = 0.01):  
    if len(self.param) == 4:
      # generate a cubic bezier curve by given control points
      p = np.array(self.param)
      c = lambda t: (1 - t)**3*p[0] + 3*t*(1 - t)**2*p[1] + 3*t**2*(1-t)*p[2] + t**3*p[3]
      self.traj = np.array([c(t) for t in np.arange(0, 1, gran)])

      for p in self.traj:
        self.point_bodies.append(self.world.CreateStaticBody(
            position=(p[0], p[1]),
            angle = 0.0,
            fixtures = WP_FD,
          )
        )
        self.point_bodies[-1].userData = 'way point'
        self.point_bodies[-1].color = (120/255, 0/255, 183/255)

      return 1
    else:
      return -1
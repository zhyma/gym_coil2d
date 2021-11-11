import sys
from math import sin, cos, pi
# from typing import ParamSpec
import numpy as np

import Box2D
from Box2D.b2 import circleShape, polygonShape, revoluteJointDef

import gym
from gym import error, spaces, utils
from gym.utils import seeding, colorize, EzPickle

from define import *
from bezier import Bezier_traj
from spiral import Spiral_traj

# from contact import find_contact
from contact import ContactDetector

class Coil2DEnv(gym.Env, EzPickle):
  metadata = {'render.modes': ['human', "rgb_array"], "video.frames_per_second":FPS}

  def __init__(self, curve = 'spiral'):
    EzPickle.__init__(self)
    # self.seed()
    self.viewer = None

    # self.g = 10
    self.g = 0
    self.gripper_weight = PHY_GRIPPER_L*PHY_GRIPPER_W*GRIPPER_DENSITY*self.g
    self.section_weight = PHY_SECT_L*PHY_SECT_W*GRIPPER_DENSITY*self.g
    self.world = Box2D.b2World(gravity=(0, -self.g))

    self.rod = None
    self.rope = None # list

    if curve == 'bezier':
      self.curve = Bezier_traj(self.world)
    else:
      self.curve = Spiral_traj(self.world)

    # the state of gripping the rope or not (-1)
    self.grabbed = -1

    # 6 low level controls: move left, move right, move up, move down, grab, release, x, y
    # TODO: update this field
    self.action_space = spaces.Discrete(6)

    self.reset()

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def _destroy(self):
    self.world.contactListener = None
    ...

  def rope_extend(self, n_sections, jpos, anchor = [PHY_SECT_L/2, 0]):
    gpos = [0, 0]
    print(n_sections)
    if n_sections > 0:
      last_anchor = anchor.copy()
      next_anchor = [-PHY_SECT_L/2, 0]
      
      last_jpos = jpos.copy()
      angle = self.rope[-1].angle
      print('extend the rope')
      print('angle='+str(angle))
      for i in range(n_sections):
        print(i, end=',')
        # create a section, spos is the section position
        spos = (last_jpos[0]+PHY_SECT_L/2*cos(angle), last_jpos[1]+PHY_SECT_L/2*sin(angle))
        new_section = self.world.CreateDynamicBody(
          position = spos,
          angle = angle,
          fixtures = [S_SECTION_FD, SECTION_FD],
          # fixtures = SECTION_FD
        )

        if i == ATTACH_NO:
          gpos = (last_jpos[0]+PHY_SECT_L/2*cos(angle), last_jpos[1]+PHY_SECT_L/2*sin(angle))

        new_section.userData = 'section_' + str(len(self.rope)-1)
        color_grad = (i%2)/2
        new_section.color = (233/255* color_grad, 196/255 * color_grad, 106/255* color_grad)
        self.rope.append(new_section)
        # create a joint between sections, attach this section to its' predecessor
        # if i < ATTACH_NO:
        if True:
          rjd = revoluteJointDef(
            bodyA = self.rope[-2],
            bodyB = self.rope[-1],
            localAnchorA = last_anchor,
            localAnchorB = next_anchor,
            enableMotor = False,
            enableLimit = True,
            lowerAngle = -pi/8,
            upperAngle = pi/8,
          )
        self.rope_joints.append(self.world.CreateJoint(rjd))
        last_anchor = [PHY_SECT_L/2, 0]
        last_jpos = [last_jpos[0]+PHY_SECT_L*cos(angle), last_jpos[1]+PHY_SECT_L*sin(angle)]
      
    print('\n')
    return gpos

  def reset(self):
    self._destroy()
    self.world.contactListener_keepref = ContactDetector(self)
    self.world.contactListener = self.world.contactListener_keepref
    self.game_over = False
    self.prev_shaping = None

    self.grabbed = -1

    # create the rod as a static body in the world
    # TODO: random starting rod position, random radius
    self.rod = self.world.CreateStaticBody(
      position=(PHY_W/2-5, PHY_H/2+3),
      angle = 0.0,
      fixtures = ROD_FD,
    )
    self.rod.userData = 'rod'
    self.rod.color = (38/255, 70/255, 83/255)

    # The gripper that pins down the starting point of the chain.
    pinned_pos = [PHY_W/2-10, PHY_H/2+5]
    self.pinned = self.world.CreateStaticBody(
      position = pinned_pos,
      angle = 0.0,
      fixtures = PINNED_FD,
    )
    self.pinned.userData = 'pinned'
    self.pinned.color = (244/255, 162/255, 97/255)

    # rope = [pinned, section_0, section_1, ...]
    self.rope = [self.pinned]
    self.grabbing_joint = None
    self.rope_joints = []
    
    # jpos stands for joint pos under the world coordinate
    last_jpos = [i for i in pinned_pos]
    # gpos_x and gpos_y stand for the gripper position (attach to a section of the rope)
    gpos = self.rope_extend(SECT_NUM, last_jpos, anchor=[0,0])

    # create the gripper
    self.gripper = self.world.CreateDynamicBody(
      position = [gpos[0]+1, gpos[1]+1],
      angle = 0.0,
      fixtures = [S_GRIPPER_FD, GRIPPER_FD],
      # fixtures = GRIPPER_FD,
      fixedRotation = True,
    )
    self.gripper.userData = 'gripper'
    self.gripper.color = (231/255, 111/255, 81/255)

    self.world.contactListener.reachable = []
    self.world.contactListener.intersect = []

    self.drawlist = [self.rod, self.gripper] + self.rope
    print('len of draw list: '+str(len(self.drawlist)))

  def step(self, action):
    intersect_pair = self.world.contactListener.find_intersect()
    # print(self.world.contactListener.reachable, end=' --> ')
    # print(self.world.contactListener.find_reachable())
    # print(self.world.contactListener.intersect, end=' --> ')
    # print(intersect_pair)
    # print('----')

    if (not action[0]==0) or (not action[1]==0):
      # move the gripper, set it to the highest prioirty
      DELTA_D = 0.1
      if self.grabbed >= 0:
        DELTA_D = 0.5
      if action[0] == -1: # left
        self.gripper.position += (-DELTA_D, 0)
      if action[0] == 1: # right
        self.gripper.position += (+DELTA_D, 0)
      if action[1] == 1: # up
        self.gripper.position += (0, +DELTA_D)
      if action[1] == -1: # down
        self.gripper.position += (0, -DELTA_D)
      
    elif action[2]==-1:
      # release the gripper (e.g., "r" is pressed)
      if self.grabbed >= 0:
        # remove the joint that connects the gripper and the rope
        self.grabbed = -1
        self.world.DestroyJoint(self.grabbing_joint)

    elif action[2]==1:
      # grab the rope
      if self.grabbed == -1 and len(self.world.contactListener.reachable) > 0:
        # make sure the gripper is not grabbing the rope, and have contact
        # gp is the grabbing point
        gp = self.world.contactListener.find_reachable()
        # grab the rope at that section
        # rope always contain one extra element (pinned)
        rjd = revoluteJointDef(
            bodyA = self.rope[gp+1],
            bodyB = self.gripper,
            localAnchorA = (0,0),
            localAnchorB = (0,0),
            enableMotor = False,
            enableLimit = False,
        )
        self.grabbing_joint = self.world.CreateJoint(rjd)
        self.grabbed = gp
      else:
        # no contact between the gripper and any sections
        print('no section within reach')

    elif action[2]==2:
        # extend sections, from current intersecting point, extend the rope
        # get the number of sections that is going to extend (a little smaller than SECT_NUM) 
        exist_extra = len(self.rope)-1 - intersect_pair[1]
        n_new = SECT_NUM - exist_extra
        print(n_new)
        if n_new > 0:
          pos = self.rope[-1].position
          angle = self.rope[-1].angle
          last_jpos = [pos[0]+PHY_SECT_L/2*cos(angle), pos[1]+PHY_SECT_L/2*sin(angle)]  
          print('number of new sections: '+str(n_new))
          print(last_jpos)
          _ = self.rope_extend(n_new, last_jpos)
          self.drawlist = [self.rod, self.gripper] + self.rope
          print('len of draw list: '+str(len(self.drawlist)))
          print('len of rope: '+str(len(self.rope)))
    else:
      pass # do nothing

    # compensate the gravity term
    self.gripper.ApplyForceToCenter((0, self.gripper_weight), True, )
    for i in self.rope:
     i.ApplyForceToCenter((0, self.section_weight), True, )

    # world.Step((dt, velocityIterations, positionIterations))
    self.world.Step(1.0 / FPS, 6 * 30, 2 * 30)
    # sections with number greater than the one being grabbed are taken as having fixed joints
    pos = (0, 0)
    angle = 0

    # update the color of sections to high light contact point and manipulation sections
    for i in range(1,len(self.rope)):
      color_grad = ((i-1)%2)/2
      # self.rope[i].color = (233/255* color_grad, 196/255 * color_grad, 106/255* color_grad)
      ...
      if i == self.grabbed+1:
        # the part being grabbed shown as green
        self.rope[i].color = GREEN
      elif i > self.grabbed+1 and self.grabbed > 0:
        # beyond the grabbing point as dark red.
        pos = self.rope[i-1].position
        angle = self.rope[i-1].angle
        self.rope[i].color = DARK_RED
        self.rope[i].position = [pos[0]+PHY_SECT_L*cos(angle), pos[1]+PHY_SECT_L*sin(angle)]
        self.rope[i].angle = angle
      else:
        # sections before the grabbing point
        # refresh section color
        color_grad = (i%2)/2
        self.rope[i].color = (233/255* color_grad, 196/255 * color_grad, 106/255* color_grad)
        if len(intersect_pair) > 0:
          self.rope[intersect_pair[0]+1].color = LIGHT_RED
          self.rope[intersect_pair[1]+1].color = LIGHT_RED

  def render(self, mode='human'):
    from gym.envs.classic_control import rendering

    if self.viewer is None:
      self.viewer = rendering.Viewer(VIEWPORT_W, VIEWPORT_H)
      self.viewer.set_bounds(0, VIEWPORT_W / SCALE, 0, VIEWPORT_H / SCALE)

    for obj in self.drawlist + self.curve.point_bodies:
      for f in obj.fixtures:
        trans = f.body.transform
        if type(f.shape) is circleShape:
          # drawing the rod
          t = rendering.Transform(translation=trans * f.shape.pos)
          # #def draw_circle(self, radius=10, res=30, filled=True, **attrs):
          self.viewer.draw_circle(f.shape.radius, 20, color=obj.color).add_attr(t)
          self.viewer.draw_circle(f.shape.radius, 20, color=obj.color, filled=False, linewidth=2).add_attr(t)
        else:
          # drawing the rope/chain or the gripper
          path = [trans * v for v in f.shape.vertices]
          self.viewer.draw_polygon(path, color=obj.color)
          path.append(path[0])
          self.viewer.draw_polyline(path, color=obj.color, linewidth=2)
    
    return self.viewer.render(return_rgb_array=mode == "rgb_array")

  def close(self):
    if self.viewer is not None:
      self.viewer.close()
      self.viewer = None

if __name__ == "__main__":
  from PIL import Image, ImageDraw
  from pyglet.window import key

  class KeyCtrl():
    def __init__(self):
        # left/right, up/down, grab/releases
        self.a = [0, 0, 0]
        self.restart = False
        self.quit = False

    def key_press(self, k, mod):
        # 0xff0d is the 'enter' key
        # a == 0: no action
        if k == 0xff0d:     self.restart = True
        if k == key.ESCAPE: self.quit = True
        if k == key.LEFT:   self.a[0] = -1
        if k == key.RIGHT:  self.a[0] =  1
        if k == key.UP:     self.a[1] =  1
        if k == key.DOWN:   self.a[1] = -1
        if k == key.G:      self.a[2] =  1 # grab
        if k == key.R:      self.a[2] = -1 # release
        if k == key.N:      self.a[2] =  2 # extend the rope

    def key_release(self, k, mod):
        if k == key.LEFT  and self.a[0] == -1: self.a[0] = 0
        if k == key.RIGHT and self.a[0] ==  1: self.a[0] = 0
        if k == key.UP    and self.a[1] ==  1: self.a[1] = 0
        if k == key.DOWN  and self.a[1] == -1: self.a[1] = 0
        if k == key.G     and self.a[2] ==  1: self.a[2] = 0
        if k == key.R     and self.a[2] == -1: self.a[2] = 0
        if k == key.N     and self.a[2] ==  2: self.a[2] = 0

  kb = KeyCtrl()

  env = Coil2DEnv()
  # env.reset()

  env.render()
  env.viewer.window.on_key_press = kb.key_press
  env.viewer.window.on_key_release = kb.key_release
  while kb.quit==False:
    env.render()
    env.step(kb.a) # take a random action?

  env.close()
import sys
import math
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
    self.rope = None
    # the total number of sections along the rope
    self.total_sects = SECT_NUM
    if curve == 'bezier':
      self.curve = Bezier_traj(self.world)
    else:
      self.curve = Spiral_traj(self.world)

    # the state of gripping the rope or not
    self.grabbed = -1
    # wherer to grab (contacting point, no matter grabbed or not)
    self.contact_section = -1

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
    last_anchor = [0, 0]
    next_anchor = [-PHY_SECT_L/2, 0]
    # jpos_world stands for joint pos under the world coordinate
    last_jpos_world = [i for i in pinned_pos]
    # gpos_x and gpos_y stand for the gripper position (attach to a section of the rope)
    gpos = (0, 0)
    # sect_x_step = PHY_SECT_L
    for i in range(SECT_NUM):
      # create a section, spos ithe section position
      spos = (last_jpos_world[0]+PHY_SECT_L/2, last_jpos_world[1])
      new_section = self.world.CreateDynamicBody(
        position = spos,
        angle = 0,
        fixtures = [S_SECTION_FD, SECTION_FD],
      )

      if i == ATTACH_NO:
        gpos = (last_jpos_world[0]+PHY_SECT_L/2, last_jpos_world[1])

      new_section.userData = 'section_' + str(i)
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
          lowerAngle = -math.pi/8,
          upperAngle = math.pi/8,
        )
      self.rope_joints.append(self.world.CreateJoint(rjd))
      last_anchor = [PHY_SECT_L/2, 0]
      last_jpos_world = [last_jpos_world[0]+PHY_SECT_L, last_jpos_world[1]]

    # create the gripper
    self.gripper = self.world.CreateDynamicBody(
      position = gpos,
      angle = 0.0,
      fixtures = [S_GRIPPER_FD, GRIPPER_FD],
      fixedRotation = True,
    )
    self.gripper.userData = 'gripper'
    self.gripper.color = (231/255, 111/255, 81/255)

    # grab the rope with the gripper
    rjd = revoluteJointDef(
        bodyA = self.rope[ATTACH_NO+1],
        bodyB = self.gripper,
        localAnchorA = (0,0),
        localAnchorB = (0,0),
        enableMotor = False,
        enableLimit = False,
    )
    self.grabbing_joint = self.world.CreateJoint(rjd)
    self.grabbed = ATTACH_NO

    self.drawlist = [self.rod, self.gripper] + self.rope

  def step(self, action):
    # print(self.contact_section)
    # print(self.rope[3])
    # self.rope[1].fixtures = SECTION_FD
    # print("type is: ",)
    # print(self.rope[1].fixtures)

    # moving by using keyboard
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

    # compensate the gravity term
    self.gripper.ApplyForceToCenter((0, self.gripper_weight), True, )
    for i in self.rope:
     i.ApplyForceToCenter((0, self.section_weight), True, )

    # "r" key is pressed --> self.a[2] == -1
    if action[2] == -1:
       if self.grabbed >= 0:
        self.grabbed = -1
        self.world.DestroyJoint(self.grabbing_joint)
        print(self.world.contactListener.loop)
        # self.rope[self.world.contactListener.loop[0]+1].color = LIGHT_RED
        # self.rope[self.world.contactListener.loop[1]+1].color = LIGHT_RED

    # "g" key is pressed --> self.a[2] == 1
    if action[2] == 1:
      # # if grabbing the rope, release it
      # if self.grabbed >= 0:
      #   self.grabbed = -1
      #   self.world.DestroyJoint(self.grabbing_joint)
      if self.grabbed == -1 and self.contact_section >= 0:
        # gp is the grabbing point
        gp = self.contact_section+1
        if gp >= len(self.rope):
          gp = len(self.rope)-1
        # grab the rope at that section
        rjd = revoluteJointDef(
            bodyA = self.rope[gp],
            bodyB = self.gripper,
            localAnchorA = (0,0),
            localAnchorB = (0,0),
            enableMotor = False,
            enableLimit = False,
        )
        self.grabbing_joint = self.world.CreateJoint(rjd)
        self.grabbed = self.contact_section

    # world.Step((dt, velocityIterations, positionIterations))
    self.world.Step(1.0 / FPS, 6 * 30, 2 * 30)
    # sections with number greater than the one being grabbed are taken as having fixed joints
    pos = (0, 0)
    angle = 0

    # TODO: add more sections if the gripper is close to the last

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
        self.rope[i].position = [pos[0]+PHY_SECT_L*math.cos(angle), pos[1]+PHY_SECT_L*math.sin(angle)]
        self.rope[i].angle = angle
      else:
        # sections before the grabbing point
        # refresh section color
        color_grad = (i%2)/2
        self.rope[i].color = (233/255* color_grad, 196/255 * color_grad, 106/255* color_grad)
        if len(self.world.contactListener.loop) > 0:
          self.rope[self.world.contactListener.loop[0]+1].color = LIGHT_RED
          self.rope[self.world.contactListener.loop[1]+1].color = LIGHT_RED
        print(self.world.contactListener.loop, end=',')

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
        if k == key.RIGHT:  self.a[0] = 1
        if k == key.UP:     self.a[1] = 1
        if k == key.DOWN:   self.a[1] = -1
        if k == key.G:      self.a[2] = 1 # grab
        if k == key.R:      self.a[2] = -1 # release

    def key_release(self, k, mod):
        if k == key.LEFT  and self.a[0] == -1: self.a[0] = 0
        if k == key.RIGHT and self.a[0] ==  1: self.a[0] = 0
        if k == key.UP    and self.a[1] ==  1: self.a[1] = 0
        if k == key.DOWN  and self.a[1] == -1: self.a[1] = 0
        if k == key.G     and self.a[2] ==  1: self.a[2] = 0
        if k == key.R     and self.a[2] == -1: self.a[2] = 0

  kb = KeyCtrl()

  env = Coil2DEnv()
  env.reset()

  env.render()
  env.viewer.window.on_key_press = kb.key_press
  env.viewer.window.on_key_release = kb.key_release
  while kb.quit==False:
    env.render()
    env.step(kb.a) # take a random action?

  env.close()
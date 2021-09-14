import sys
import math
import numpy as np

import Box2D
from Box2D.b2 import (
    edgeShape,
    circleShape,
    fixtureDef,
    polygonShape,
    revoluteJointDef,
    contactListener,
)

import gym
from gym import error, spaces, utils
from gym.utils import seeding, colorize, EzPickle

import Box2D

FPS = 50
SCALE = 30.0  # affects how fast-paced the game is, forces should be adjusted as well

VIEWPORT_W = 600
VIEWPORT_H = 400

PHY_W = VIEWPORT_W / SCALE
PHY_H = VIEWPORT_H / SCALE

# all numbers defined here are measured in pixels
# will not fullow OpenAI's naming convention, 
# PX for measuring in pixels
# PHY for measuring in meters
# PHY_* = PX_*/SCALE
PX_GRIPPER_L = 30
PX_GRIPPER_W = 30
PX_GRIPPER_POLY = [(-PX_GRIPPER_L/2, PX_GRIPPER_W/2), (PX_GRIPPER_L/2, PX_GRIPPER_W/2),\
                  (PX_GRIPPER_L/2, -PX_GRIPPER_W/2), (-PX_GRIPPER_L/2, -PX_GRIPPER_W/2)]
# number of sections along the chain
SECT_NUM = 16 # the number of chains
PX_SECT_L = 30 # the length of a piece of the chains
PX_SECT_W = 6 # the width of a piece of the chains
PX_SECT_POLY = [(-PX_SECT_L/2, PX_SECT_W/2), (PX_SECT_L/2, PX_SECT_W/2),\
            (PX_SECT_L/2, -PX_SECT_W/2), (-PX_SECT_L/2, -PX_SECT_W/2)]

PHY_SECT_L = PX_SECT_L/SCALE

# a section of the chain
# FD stands for fixtureDef
SECT_FD = fixtureDef(
  shape=polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_SECT_POLY]),
  density=5.0,
  friction=1,
  categoryBits=0x04,
  maskBits=0x02,  # collide only with rod
  restitution=0,
)

PINNED_FD = fixtureDef(
  shape = polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_GRIPPER_POLY]),
  density = 1,
  friction = 1, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x04,
  maskBits = 0x02, # the gripper collides with the rod but not the rope
)

GRIPPER_FD = fixtureDef(
  shape = polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_GRIPPER_POLY]),
  density = 1,
  friction = 1, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x04,
  maskBits = 0x02, # the gripper collides with the rod but not the rope
)

ROD_FD = fixtureDef(
  shape = circleShape(pos = (0, 0), radius = 2),
  density = 1,
  friction = 1, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x02,
  maskBits = 0x04, # the rod collides with both the rope and the gripper
)

class ContactDetector(contactListener):
    def __init__(self, env):
        contactListener.__init__(self)
        self.env = env

    def BeginContact(self, contact):
        ...

    def EndContact(self, contact):
        ...

class Coil2DEnv(gym.Env, EzPickle):
  metadata = {'render.modes': ['human', "rgb_array"], "video.frames_per_second":FPS}

  def __init__(self):
    EzPickle.__init__(self)
    # self.seed()
    self.viewer = None

    self.world = Box2D.b2World()
    self.rod = None
    self.rope = None

    # 6 low level controls: move left, move right, move up, move down, grab, release
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
    
    # create the rod as a static body in the world
    # TODO: random starting rod position, random radius
    self.rod = self.world.CreateStaticBody(
      position=(PHY_W/2-5, PHY_H/2+3),
      angle = 0.0,
      fixtures = ROD_FD,
    )
    self.rod.color = (38/255, 70/255, 83/255)

    # The gripper that pins down the starting point of the chain.
    pinned_pos = [PHY_W/2-8, PHY_H/2+5]
    self.pinned = self.world.CreateStaticBody(
      position = pinned_pos,
      angle = 0.0,
      fixtures = PINNED_FD,
    )
    self.pinned.color = (244/255, 162/255, 97/255)

    self.sections = [self.pinned]
    self.joints = []
    last_anchor = [0, 0]
    next_anchor = [-PHY_SECT_L/2, 0]
    # jpos_world stands for joint pos under the world coordinate
    last_jpos_world = [i for i in pinned_pos]
    sect_x_step = PHY_SECT_L
    for i in range(SECT_NUM):
      # create a section
      new_section = self.world.CreateDynamicBody(
        position = (last_jpos_world[0]+sect_x_step/2, last_jpos_world[1]),
        angle = 0,
        fixtures = SECT_FD,
      )
      # place everything on the ground
      new_section.gravityScale = 0
      color_grad = i/SECT_NUM/2+0.5
      new_section.color = (233/255* color_grad, 196/255 * color_grad, 106/255* color_grad)
      self.sections.append(new_section)
      # attach this section to its' predecessor
      rjd = revoluteJointDef(
        bodyA = self.sections[-2],
        bodyB = self.sections[-1],
        localAnchorA = last_anchor,
        localAnchorB = next_anchor,
        enableMotor = False,
        enableLimit = False,
        # enableLimit = True,
        # lowerAngle = -math.pi/4,
        # upperAngle = math.pi/4,
      )
      self.joints.append(self.world.CreateJoint(rjd))
      last_anchor = [PHY_SECT_L/2, 0]
      last_jpos_world = [last_jpos_world[0]+sect_x_step, last_jpos_world[1]]

    # TODO: create the links of rope as dynamic bodies
    self.gripper = self.world.CreateDynamicBody(
      position = (last_jpos_world[0]+sect_x_step, last_jpos_world[1]),
      angle = 0.0,
      fixtures = GRIPPER_FD,
    )
    self.gripper.color = (231/255, 111/255, 81/255)
    # self.gripper.gravityScale = 0

    rjd = revoluteJointDef(
        bodyA = self.sections[-1],
        bodyB = self.gripper,
        localAnchorA = last_anchor,
        localAnchorB = (0,0),
        enableMotor = False,
        enableLimit = False,
        # enableLimit = True,
        # lowerAngle = -math.pi/4,
        # upperAngle = math.pi/4,
    )
    self.joints.append(self.world.CreateJoint(rjd))

    self.drawlist = [self.rod, self.gripper] + self.sections

  def step(self, action):
    self.world.Step(1.0 / FPS, 6 * 30, 2 * 30)
    ...

    
  def render(self, mode='human'):
    from gym.envs.classic_control import rendering

    if self.viewer is None:
      self.viewer = rendering.Viewer(VIEWPORT_W, VIEWPORT_H)
      self.viewer.set_bounds(0, VIEWPORT_W / SCALE, 0, VIEWPORT_H / SCALE)

    for obj in self.drawlist:
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
  env = Coil2DEnv()
  env.reset()
  for _ in range(300):
    env.render()
    env.step(env.action_space.sample()) # take a random action?
  env.close()
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

# numbers defined here are measured in pixels
GRIPPER_POLY = [(-15, 15), (15, 15), (15, -15), (-15, -15)]
# number of sections along the chain
SECT_NUM = 12 # the number of chains
SECT_L = 30 # the length of a piece of the chains
SECT_W = 6 # the width of a piece of the chains
SECT_POLY = [(-SECT_L/2, SECT_W/2), (SECT_L/2, SECT_W/2), (SECT_L/2, -SECT_W/2), (-SECT_L/2, -SECT_W/2)]

# a section of the chain
SECT = fixtureDef(
    shape=polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in SECT_POLY]),
    density=5.0,
    friction=1,
    categoryBits=0x04,
    maskBits=0x02,  # collide only with rod
    restitution=0,
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

    W = VIEWPORT_W / SCALE
    H = VIEWPORT_H / SCALE

    # create the rod as a static body in the world
    # TODO: random starting rod position, random radius
    self.rod = self.world.CreateStaticBody(
      position=(W/2, H/2),
      angle = 0.0,
      fixtures = fixtureDef(
          shape = circleShape(pos = (0, 0), radius = 0.5),
          density = 1,
          friction = 1, # very high friction
          restitution = 0, # no restitution
          categoryBits = 0x02,
          maskBits = 0x04, # the rod collides with both the rope and the gripper
        ),
    )
    self.rod.color = (0.15, 0.27, 0.32)

    # TODO: create the links of rope as dynamic bodies
    self.gripper = self.world.CreateDynamicBody(
      position = (W/2-2, H/2-2),
      angle = 0.0,
      fixtures = fixtureDef(
        shape = polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in GRIPPER_POLY]),
        density = 1,
        friction = 1, # very high friction
        restitution = 0, # no restitution
        categoryBits = 0x04,
        maskBits = 0x02, # the gripper collides with the rod but not the rope
      )
    )
    self.gripper.color = (0.91, 0.77, 0.42)

    self.sections = []
    self.joints = []
    # TODO: create the gripper as a dynamic body
    for i in range(SECT_NUM):
      section = self.world.CreateDynamicBody(
        position = (i*2, i*2),
        angle = 0,
        fixtures = SECT,
      )
      section.color = (0.91, 0.44, 0.32)
      self.sections.append(section)


    # self.drawlist = [self.rod, self.gripper] + self.rope.chains
    self.drawlist = [self.rod, self.gripper] + self.sections

  def step(self, action):
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
  for _ in range(1000):
    env.render()
    # env.step(env.action_space.sample()) # take a random action?
  env.close()
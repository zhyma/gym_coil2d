import sys
import math
# from typing import ParamSpec
import numpy as np
# from bezier_3 import bezier_3

import Box2D
from Box2D.b2 import (
    # edgeShape,
    circleShape,
    fixtureDef,
    polygonShape,
    revoluteJointDef,
    contactListener,
)

import gym
from gym import error, spaces, utils
from gym.utils import seeding, colorize, EzPickle

FPS = 50
SCALE = 30.0  # affects how fast-paced the game is, forces should be adjusted as well

VIEWPORT_W = 800
VIEWPORT_H = 800

PHY_W = VIEWPORT_W / SCALE
PHY_H = VIEWPORT_H / SCALE

# all numbers defined here are measured in pixels
# will not fullow OpenAI's naming convention, 
# PX for measuring in pixels
# PHY for measuring in meters
# PHY_??? = PX_???/SCALE
PX_GRIPPER_L = 30
PX_GRIPPER_W = 30
PX_GRIPPER_POLY = [(-PX_GRIPPER_L/2, PX_GRIPPER_W/2), (PX_GRIPPER_L/2, PX_GRIPPER_W/2),\
                   (PX_GRIPPER_L/2, -PX_GRIPPER_W/2), (-PX_GRIPPER_L/2, -PX_GRIPPER_W/2)]
# number of sections along the chain
SECT_NUM = 36 # the number of chains
PX_SECT_L = 30 # the length of a piece of the chains
PX_SECT_W = 6 # the width of a piece of the chains
PX_SECT_POLY = [(-PX_SECT_L/2, PX_SECT_W/2), (PX_SECT_L/2, PX_SECT_W/2),\
                (PX_SECT_L/2, -PX_SECT_W/2), (-PX_SECT_L/2, -PX_SECT_W/2)]

# gripper initially clamped to section No. ?
ATTACH_NO = 18

PHY_SECT_L = PX_SECT_L/SCALE
PHY_SECT_W = PX_SECT_W/SCALE

PHY_GRIPPER_L = PX_GRIPPER_L/SCALE
PHY_GRIPPER_W = PX_GRIPPER_W/SCALE

GRIPPER_DENSITY = 0.1
SECTION_DENSITY = 0.1

GREEN = (30/255, 120/255, 30/255)
DARK_RED = (120/255, 30/255, 30/255)
LIGHT_RED = (150/255, 0/255, 0/255)

# a section of the chain
# FD stands for fixtureDef
SECTION_FD = fixtureDef(
  shape=polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_SECT_POLY]),
  density = SECTION_DENSITY,
  friction = 0,
  categoryBits = 0x04,
  maskBits = 0x10 + 0x02,  # collide only with rod
  restitution = 0,
)

PINNED_FD = fixtureDef(
  shape = polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_GRIPPER_POLY]),
  density = GRIPPER_DENSITY,
  friction = 0, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x08,
  maskBits = 0x08 + 0x02, # the gripper collides with the rod but not the rope
)

GRIPPER_FD = fixtureDef(
  shape = polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_GRIPPER_POLY]),
  density = GRIPPER_DENSITY,
  friction = 1, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x08,
  maskBits = 0x08 + 0x02, # the gripper collides with the rod but not the rope
)

# sensor has no density
# define a sensor for the gripper
S_GRIPPER_FD = fixtureDef(
  shape = polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_GRIPPER_POLY]),
  categoryBits = 0x10,
  maskBits = 0x04, # the gripper collides with the rod but not the rope
  isSensor = True,
)
# define a sensor for the rope
S_SECTION_FD = fixtureDef(
  shape = polygonShape(vertices=[(x / SCALE, y / SCALE) for x, y in PX_SECT_POLY]),
  categoryBits = 0x16,
  maskBits = 0x16, # sections only collides with other sections
  isSensor = True,
)

ROD_FD = fixtureDef(
  shape = circleShape(pos = (0, 0), radius = 2),
  density = 1.0,
  friction = 1, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x02,
  maskBits = 0x08 + 0x04 + 0x02, # the rod collides with both the rope and the gripper
)

# Bezier curve control point
BP_FD = fixtureDef(
  shape = circleShape(pos = (0, 0), radius = 0.5),
  density = 0,
  friction = 0, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x00,
  maskBits = 0x00, # the rod collides with both the rope and the gripper
)

# Bezier curve way points (sampling points)
WP_FD = fixtureDef(
  shape = circleShape(pos = (0, 0), radius = 0.1),
  density = 0,
  friction = 0, # very high friction
  restitution = 0, # no restitution
  categoryBits = 0x00,
  maskBits = 0x00, # the rod collides with both the rope and the gripper
)

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

class Spiral_traj():
  def __init__(self, world):
    # parameters: direction (cw/ccw), \theta_offset (t_os), \theta_0 (t_0), \theta_e (t_e)
    self.para_len = 4
    self.param = []
    self.traj = []
    self.point_bodies = []
    self.world = world
    self.ctrl_start = Box2D.b2Vec2(0,0)
    self.rod_pos = Box2D.b2Vec2(0,0)

  def clear_bodies(self, bodies):
    while len(bodies) > 0:
      self.world.DestroyBody(bodies[-1])
      del(bodies[-1])
  
  def clean(self):
    self.param = []
    self.traj = []
    self.clear_bodies(self.point_bodies)

  def traj_gen(self, gran = 0.1):
    self.clean()
    n = -1/2
    x_r = self.rod_pos[0]
    y_r = self.rod_pos[1]
    x_g = self.ctrl_start[0]
    y_g = self.ctrl_start[1]
    t_0 = 0.2 # self.param[1]
    t_e = t_0 + np.pi*2 # self.param[2]
    # t_os = 0
    if x_g-x_r == 0:
      if y_g < y_r:
        t_os = 3/2*np.pi
      else:
        t_os = np.pi/2
    else:
      t_os = np.arctan((y_g-y_r)/(x_g-x_r))

    r_0 = np.sqrt((x_r-x_g)**2+(y_r-y_g)**2)
    a = r_0/np.power(t_0, n)
    r_t = [a*np.power((t_0+t), n) for t in np.arange(0, t_e-t_0, gran)]
    for dt in np.arange(0, t_e-t_0, gran):
      r = a*np.power((t_0+dt), n)
      t = dt + t_os
      x = r*np.cos(t) + x_r
      y = r*np.sin(t) + y_r
      self.traj.append([x,y])

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

class ContactDetector(contactListener):
    def __init__(self, env):
      contactListener.__init__(self)
      # contact queue
      # a queue that saves all the contacted sections.
      # sort by number
      self.que = []
      self.env = env
      self.loop = []

    def BeginContact(self, contact):
      self._contact(contact, True)
      ...

    def EndContact(self, contact):
      self._contact(contact, False)
      ...

    def _contact(self, contact, begin):
      section = None
      gripper = None
      u1 = contact.fixtureA.body.userData
      u2 = contact.fixtureB.body.userData
      
      # contact is between the gripper and sections
      if ('gripper' in u1 and 'section' in u2) or ('section' in u1 and 'gripper' in u2):
        if 'gripper' in u1:
          gripper = u1
          section = u2
        elif 'section' in u1:
          gripper = u2
          section = u1
        else:
          return

        sect = int(section.split('_')[1])
        if begin:
          # BeginContact, insert the contacted section into the queue.
          if not sect in self.que:
            if len(self.que) == 0:
              self.que = [sect]
            else:
              idx = 0
              while idx < len(self.que) and self.que[idx] < sect:
                idx += 1
              self.que.insert(idx, sect)
            # print(self.que, ', ', len(self.que))
        else:
          # EndContact, remove the section from the queue
          if sect in self.que:
            self.que.remove(sect)

        if len(self.que) == 0:
          self.env.contact_section = -1
        else:
          self.env.contact_section = self.que[len(self.que)//2]

      # if contact is between sections
      if ('section' in u1) and ('section' in u2):
        sect_1 = int(u1.split('_')[1])
        sect_2 = int(u2.split('_')[1])
        if abs(sect_1-sect_2) > 3:
          self.loop = [sect_1, sect_2]
          # print("close loop detected")

class Coil2DEnv(gym.Env, EzPickle):
  metadata = {'render.modes': ['human', "rgb_array"], "video.frames_per_second":FPS}

  def __init__(self):
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
    # self.curve = Bezier_traj(self.world)
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
    self.curve.rod_pos = self.rod.position

    # The gripper that pins down the starting point of the chain.
    pinned_pos = [PHY_W/2-8, PHY_H/2+5]
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
          # enableLimit = False,
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

    # If you want to generate a Bezier curve,
    # the position of th gripper will be the first control point
    self.curve.ctrl_start = self.gripper.position

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
      self.gripper.position += (0, +DELTA_D*2)
    if action[1] == -1: # down
      self.gripper.position += (0, -DELTA_D)

    # compensate the gravity term
    self.gripper.ApplyForceToCenter((0, self.gripper_weight), True, )
    for i in self.rope:
      i.ApplyForceToCenter((0, self.section_weight), True, )

    # "r" key is pressed, or, self.a[2] == -1
    if action[2] == -1:
       if self.grabbed >= 0:
        self.grabbed = -1
        self.world.DestroyJoint(self.grabbing_joint)
        print(self.world.contactListener.loop)
        self.rope[self.world.contactListener.loop[0]+1].color = LIGHT_RED
        self.rope[self.world.contactListener.loop[1]+1].color = LIGHT_RED

    # "g" key is pressed, or, self.a[2] == 1
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
    for i in range(1,len(self.rope)):
      color_grad = ((i-1)%2)/2
      # self.rope[i].color = (233/255* color_grad, 196/255 * color_grad, 106/255* color_grad)
      ...
      if i == self.grabbed+1:
        self.rope[i].color = GREEN
      elif i > self.grabbed+1 and self.grabbed > 0:
        pos = self.rope[i-1].position
        angle = self.rope[i-1].angle
        self.rope[i].color = DARK_RED
        self.rope[i].position = [pos[0]+PHY_SECT_L*math.cos(angle), pos[1]+PHY_SECT_L*math.sin(angle)]
        self.rope[i].angle = angle
      else:
        ...

    if action[3] > -1 and action[4] > -1:
      # self.curve.place_ctrl_param(action[3], action[4])
      self.curve.traj_gen()
      action[3] = -1
      action[4] = -1
      # print(self.curve.param)

    
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

  class Interactive():
    def __init__(self):
        # left/right, up/down, grab/releases
        self.a = [0, 0, 0, -1, -1]
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

    def mouse_press(self, x, y, button, modifiers):
      pass

    def mouse_release(self, x, y, button, modifiers):
      self.pos = [x, y]
      self.a[3] = x
      self.a[4] = y
      # print(self.pos)

  kb = Interactive()

  env = Coil2DEnv()
  env.reset()
  images = []
  # anime = True
  anime = False

  env.render()
  env.viewer.window.on_key_press = kb.key_press
  env.viewer.window.on_key_release = kb.key_release
  env.viewer.window.on_mouse_press = kb.mouse_press
  env.viewer.window.on_mouse_release = kb.mouse_release
  while kb.quit==False:
    env.render()
    if anime:
      im = Image.fromarray(env.viewer.get_array())
      images.append(im)

    env.step(kb.a) # take a random action?

  if anime:
    images[0].save('test.gif', save_all=True, append_images=images[1:],optimize=False,duration=20,loop=0)
  env.close()
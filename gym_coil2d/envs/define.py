import Box2D
from Box2D.b2 import (
    # edgeShape,
    circleShape,
    fixtureDef,
    polygonShape,
    revoluteJointDef,
    contactListener,
)

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
ATTACH_NO = 22

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
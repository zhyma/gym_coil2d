import numpy as np
from envs.coil2d_env import Coil2DEnv
import gym

import curve.spiral as spiral

def dist(p1, p2):
  return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

if __name__ == "__main__":
  from PIL import Image, ImageDraw
  from pyglet.window import key

  game_play = True

  class Interactive():
    def __init__(self):
      # left/right, up/down, grab/releases
      self.a = [[0,0], 0, -1]
      self.restart = False
      self.quit = False
      self.start = False

    def key_press(self, k, mod):
      # 0xff0d is the 'enter' key
      # a == 0: no action
      if k == 0xff0d:     self.restart = True
      if k == key.ESCAPE: self.quit = True
      if k == key.S:      self.start = True

  kb = Interactive()

  env = gym.make('gym_coil2d:coil2d-v0')
  # env = Coil2DEnv()
  env.reset()

  def noop():
    env.render()
    env.step([[0, 0], 0, 0])

  def one_loop(attach_no):
    for i in range(10):
      env.render()
      dx = env.rope[attach_no].position[0] - env.gripper.position[0]
      dy = env.rope[attach_no].position[1] - env.gripper.position[1]
      env.step([[dx, dy], 0, 0])
    env.render()
    env.step([[0, 0], 1, 0])
    # generate one loop
    # 0 for CCW / 1 for CW, t_0 for starting angle, t_e for leaving angle
    param = [1, 0.2, np.pi*2]
    # generate trajectory
    traj = spiral.traj_gen(param, env.rod.position, env.gripper.position)
    env.step([[0, 0], 0, traj])
    # execute trajectory
    for i in traj:
      for j in range(3):
        env.render()
        dx = i[0] - env.gripper.position[0]
        dy = i[1] - env.gripper.position[1]
        env.step([[dx, dy], 0, 0])

    for i in range(30):
      env.render()
      dx = traj[-1][0] - env.gripper.position[0]
      dy = traj[-1][1] - env.gripper.position[1]
      env.step([[dx, dy], 0, 0])
    # clear gui trajectory
    env.render()
    env.step([[0, 0], 0, -1])

    for i in range(10):
      noop()
    # release the gripper
    env.render()
    env.step([[0, 0], -1, 0])
    # extend the rope
    env.render()
    env.step([[0, 0], 2, 0])
    for i in range(10):
      noop()
    ...

  env.render()
  env.viewer.window.on_key_press = kb.key_press

  # initializing the environment
  for i in range(30):
    noop()

  while kb.quit==False:
    env.render()
    if kb.start:
      # start the demo logic
      noop()
      one_loop(22)

      # grasp the next one
      one_loop(37)

      one_loop(52)

      kb.start = False
    else:
      noop()

  env.close()
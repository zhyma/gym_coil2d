import numpy as np
from coil2d_env import Coil2DEnv

def dist(p1, p2):
  return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def motion_planner():
  ...

def one_loop(env, param):
  env.curve.traj_gen(param, env.rod.position, env.gripper.position)
  print('traj ready')
  for p in env.curve.traj:
    time_out = 0
    while time_out < 5:
      a = [0]*3
      x = env.gripper.position[0]
      y = env.gripper.position[1]
      d = dist([x, y], p)
        
      if d > 0.14:
        if p[0] < x-0.1:
          a[0] = -1
        elif p[0] > x+0.1:
          a[0] = 1
        if p[1] < y-0.1:
          a[1] = -1
        elif p[1] > y+0.1:
          a[1] = 1

        # check the gripper's previous position and current position
        if dist(env.gripper.position, [x, y]) < 0.08:
          # not moving for some reason
          time_out += 1
        else:
          pass
      else:
        # when d <= -0.14
        time_out = 10

      env.step(a)
      env.render()

if __name__ == "__main__":
  from PIL import Image, ImageDraw
  from pyglet.window import key

  class Interactive():
    def __init__(self):
      # left/right, up/down, grab/releases
      self.a = [0, 0, 0, -1, -1]
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

  env = Coil2DEnv(curve='spiral')
  env.reset()

  env.render()
  env.viewer.window.on_key_press = kb.key_press

  # initializing the environment
  # PRESS S TO START!
  for i in range(30):
    env.render()
    env.step([0]*3)

  while kb.start == False:
    env.render()
    env.step([0]*3)
    
  # 0 for CCW / 1 for CW, t_0 for starting angle, t_e for leaving angle
  one_loop(env, [1, 0.2, np.pi*2])
  # release the gripper, 
  
  r = env.step([0, 0, -1, -1, -1])

  # wait to quit
  while kb.quit==False:
    env.render()
    env.step([0]*3) 

  env.close()
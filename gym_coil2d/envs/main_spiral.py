import numpy as np
from coil2d_env import Coil2DEnv

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
    env.micro_step([0]*3)

  while kb.start == False:
    env.render()
    env.micro_step([0]*3)
    
  # 0 for CCW / 1 for CW, t_0 for starting angle, t_e for leaving angle
  env.macro_step([1, 0.2, np.pi*2])

  # wait to quit
  while kb.quit==False:
    env.render()
    env.micro_step([0]*3) 

  env.close()
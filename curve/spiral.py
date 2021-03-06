import numpy as np
import matplotlib.pyplot as plt

# from define import *

def traj_gen(param, rod_pos, gripper_pos, gran = 0.1):
  """

  Generate a spiral curve (Lituus) with given parameters
  The line connected the center of the rod and the center of the gripper
  is the starting position.

  Args:
      param[0]: direction, 0 for CCW (angle > 0), 1 for CW (angle < 0)
      param[1]: t_0, \theta_0 as the starting position under the spiral coordinate
      param[2]: t_e, \theta_e as the end position under the spiral coordinate

  Return:
      1 as generating the trajectory successfully.
  """
  # lituus spiral curve
  n = -1/2
  x_r = rod_pos[0]
  y_r = rod_pos[1]
  x_g = gripper_pos[0]
  y_g = gripper_pos[1]
  # t_os, a.k.a \theta_offset, 
  # the angle offset between the starting position and the box2d coordinate.
  # The offset between the spiral coordinate and the world coordinate
  t_0 = param[1] # 0.2
  t_e = param[2] # t_0 + np.pi*2

  traj = []

  if param[0] == 0:
    # go around the rod in a CCW manner (and angles are positive)
    # t_os: theta_offset
    pass
  #   # haven't test yet
  #   if x_g-x_r == 0:
  #     if y_g < y_r:
  #       t_os = 3/2*np.pi
  #     else:
  #       t_os = np.pi/2
  #   else:
  #     t_os = np.arctan((y_g-y_r)/(x_g-x_r))

  #   r_0 = np.sqrt((x_r-x_g)**2+(y_r-y_g)**2)
  #   a = r_0/np.power(t_0, n)
  #   for dt in np.arange(0, t_e-t_0, gran):
  #     r = a*np.power((t_0+dt), n)
  #     t = dt + t_os
  #     x = r*np.cos(t) + x_r
  #     y = r*np.sin(t) + y_r
  #     self.traj.append([x,y])
  else:
    # go around the rod in a CW manner (and angles are negative)
    if x_g-x_r == 0:
      if y_g < y_r:
        t_os = 3/2*np.pi
      else:
        t_os = np.pi/2
    else:
      t_os = np.arctan((y_g-y_r)/(x_g-x_r))

    r_0 = np.sqrt((x_r-x_g)**2+(y_r-y_g)**2)
    a = r_0/np.power(t_0, n)
    for dt in np.arange(0, t_e-t_0, gran):
      r = a*np.power((t_0+dt), n)
      t = -dt + t_os
      x = r*np.cos(t) + x_r
      y = r*np.sin(t) + y_r
      traj.append([x,y])

  return traj
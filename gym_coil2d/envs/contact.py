import gym
from gym import error, spaces, utils
from gym.utils import seeding, colorize, EzPickle

import numpy as np

from Box2D.b2 import contactListener

def dist(p1, p2):
  return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

class ContactDetector(contactListener):
    def __init__(self, env):
      contactListener.__init__(self)
      # contact queue
      # a queue that saves all the contacted sections.
      # sort by number
      self.env = env
      self.loop = []
      self.reachable = []
      self.intersect = []

    def BeginContact(self, contact):
      # self._contact(contact, True)
      # print('****')
      u1 = contact.fixtureA.body.userData
      u2 = contact.fixtureB.body.userData
      # print('['+u1+', '+u2+'], ', end='')
      if u1=='rod' or u2=='rod':
        # if the rod is involved, ignore.
        pass
      elif ('section' in u1) and (u2 == 'gripper'):
        # if the contact is between the gripper and the section
        s = int(u1.split('_')[1])
        if not s in self.reachable:
          self.reachable.append(s)
      elif (u1 == 'gripper') and ('section' in u2):
        # if the contact is between the gripper and the section
        s = int(u2.split('_')[1])
        if not s in self.reachable:
          self.reachable.append(s)
      elif ('section' in u1) and ('section' in u2):
        # section and section
        s1 = int(u1.split('_')[1])
        s2 = int(u2.split('_')[1])
        # ignore adjacent sections
        s = []
        if s1+1 < s2:
          s = [s1, s2]
        elif s1 > s2+1:
          s = [s2, s1]
        if len(s)>0 and (not s in self.intersect):
          self.intersect.append(s)

      # print('====')

    def EndContact(self, contact):
      # self._contact(contact, False)
      u1 = contact.fixtureA.body.userData
      u2 = contact.fixtureB.body.userData
      if ('section' in u1) and (u2 == 'gripper'):
        # if the contact is between the gripper and the section
        s = int(u1.split('_')[1])
        if s in self.reachable:
          self.reachable.remove(s)
      elif (u1 == 'gripper') and ('section' in u2):
        # if the contact is between the gripper and the section
        s = int(u2.split('_')[1])
        if s in self.reachable:
          self.reachable.remove(s)
      elif ('section' in u1) and ('section' in u2):
        # section and section
        s1 = int(u1.split('_')[1])
        s2 = int(u2.split('_')[1])
        s = [s1, s2]
        if s1 > s2+1:
          s=[s2, s1]
        if s in self.intersect:
          self.intersect.remove(s)

    def _contact(self, contact, begin):
      ...

    def find_reachable(self):
      # find out the approaching point
      # find the section with the closest distance to the gripper
      grab_point = -1
      dist_min = 1e5
      if len(self.reachable) > 0:
        p2 = self.env.gripper.position
        for s in self.reachable:
          p1 = self.env.rope[s+1].position
          d = dist(p1,p2)
          if d < dist_min:
            grab_point = s
            dist_min = d

      return grab_point

    # find out the loop
    def find_intersect(self):
      new_loop = []
      frontier = -1
      dist = 0
      if len(self.intersect) > 0:
        for p in self.intersect:
          # p stands for pairs
          if p[1] > frontier:
            # primary condition: the farest possible section
            new_loop = p
            dist = p[1]-p[0]
          elif p[1] == frontier:
            # secondary condition: the pair with the farest distance
            if p[1]-p[0] > dist:
              new_loop = p
              dist = p[1]-p[0]
          else:
            pass

      return new_loop
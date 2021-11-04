import gym
from gym import error, spaces, utils
from gym.utils import seeding, colorize, EzPickle

from Box2D.b2 import contactListener

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
            # make sure it is a loop (interval is long enough)
          self.loop = [sect_1, sect_2]
          # print("close loop detected")
        else:
            self.loop = []
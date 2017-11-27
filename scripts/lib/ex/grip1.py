#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move fingers of gripper to a target position.
  Usage: ex.grip1 ARM, POS
    ARM: RIGHT or LEFT
    POS: Position in meter. 0 is closed
      Range of POS (RIGHT): [0.00,0.037]
      Range of POS (LEFT): [0.0,0.0855]
  '''
def Run(ct,*args):
  arm= args[0]
  pos= args[1]
  ct.robot.MoveGripper(pos, arm=arm)
  print 'Moving {arm}-gripper to {pos}'.format(arm=LRToStr(arm), pos=pos)

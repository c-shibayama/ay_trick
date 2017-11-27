#!/usr/bin/python
from core_tool import *
def Help():
  return '''Example of inverse kinematics.
  Usage: ex.ik1 ARM, POSE
    ARM: RIGHT or LEFT
    POSE: Array of target pose = [x,y,z,qx,qy,qz,qw] (position, quaternion)
  '''
def Run(ct,*args):
  arm= args[0]
  x_trg= args[1]
  q= ct.robot.IK(x_trg, arm=arm)
  print 'IK solution= {q}'.format(q=q)

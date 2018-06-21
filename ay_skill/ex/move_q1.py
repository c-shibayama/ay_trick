#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm to target joint angles.
  Warning: Be careful to moving area of robot.
  Usage: ex.move_q1 ARM, D_ANGLES
    ARM: RIGHT or LEFT, default=ct.robot.Arm
    D_ANGLES: Array of displacement of joint-angles. [0.0]*DoF == current angles
  '''
def Run(ct,*args):
  arm= args[0]
  dq= args[1]
  q= ct.robot.Q(arm=arm)  #Current joint angles
  q_trg= [q[d]+dq[d] for d in range(ct.robot.DoF())]  #Target
  print 'Moving {arm}-arm to {q}'.format(arm=LRToStr(arm), q=q_trg)
  ct.robot.MoveToQ(q_trg, dt=4.0, arm=arm)

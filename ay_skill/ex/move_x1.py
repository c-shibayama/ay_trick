#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm to target end-effector pose.
  Warning: Be careful to moving area of robot.
  Usage: ex.move_x1 ARM, D_POS
    ARM: RIGHT or LEFT, default=ct.robot.Arm
    D_POS: Array of displacement of end-effector position (x,y,z). [0.0]*3 current position
  '''
def Run(ct,*args):
  arm= args[0]
  dp= args[1]
  x= list(ct.robot.FK(arm=arm))  #Current end-effector pose
  x_trg= [x[d]+dp[d] for d in range(3)] + x[3:]  #Target
  print 'Moving {arm}-arm to {x}'.format(arm=LRToStr(arm), x=x_trg)
  ct.robot.MoveToX(x_trg, dt=4.0, arm=arm)

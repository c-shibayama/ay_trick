#!/usr/bin/python
from core_tool import *
def Help():
  return '''Get end-effector pose by solving forward kinematics.
  Note: pose = [x,y,z,qx,qy,qz,qw] (position, quaternion)
  Usage: ex.get_x1 [ARM [, ANGLES]]
    ARM: RIGHT or LEFT. default=ct.robot.Arm
    ANGLES: Array of joint angles (7 elements). default=ct.robot.Q(arm=ARM)
  '''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else ct.robot.Arm
  q= args[1] if len(args)>1 else ct.robot.Q(arm=arm)
  x= ct.robot.FK(q=q, arm=arm)
  print 'End-effector pose of {arm}-arm= {x}'.format(arm=ct.robot.ArmStr(arm), x=list(x))

#!/usr/bin/python
from core_tool import *
def Help():
  return '''Get current joint angles.
  Usage: ex.get_q1 [ARM]
    ARM: RIGHT or LEFT, default=ct.robot.Arm
  '''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else ct.robot.Arm
  q= ct.robot.Q(arm=arm)  #Get current joint angles
  print 'Joint angles of {arm}-arm= {q}'.format(arm=ct.robot.ArmStr(arm), q=q)

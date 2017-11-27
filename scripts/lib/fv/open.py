#!/usr/bin/python
from core_tool import *
def Help():
  return '''Opening gripper with post process.
  Usage: fv.open [ARM]
    ARM: RIGHT or LEFT. Default: ct.robot.Arm'''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else ct.robot.Arm
  ct.Run('fv.grasp','off',arm)
  ct.Run('fv.hold','off',arm)
  ct.Run('fv.openif','off',arm)
  ct.robot.OpenGripper(arm)
  ct.Run('fv.finger3','start_detect_obj',arm)


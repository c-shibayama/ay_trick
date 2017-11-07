#!/usr/bin/python
from core_tool import *
def Help():
  return '''Opening gripper with post process.
  Usage: fv.vs_open [ARM]
    ARM: RIGHT or LEFT. Default: ct.robot.Arm'''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else ct.robot.Arm
  ct.Run('fv.vs_grasp','off',arm)
  ct.Run('fv.vs_hold','off',arm)
  ct.Run('fv.vs_openif','off',arm)
  ct.robot.OpenGripper(arm)
  ct.Run('fv.vs_finger3','start_detect_obj',arm)


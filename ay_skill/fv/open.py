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
  #ct.robot.OpenGripper(arm)
  ct.robot.MoveGripper(pos=ct.robot.GripperPos(arm)+0.02, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm])
  ct.Run('fv.fv','start_detect_obj',arm)


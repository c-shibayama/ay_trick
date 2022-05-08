#!/usr/bin/python
from core_tool import *
def Help():
  return '''Opening gripper with post process.
  Usage: fv.open [ARM [, G_POS [, BLOCKING]]]
    ARM: RIGHT or LEFT. Default: ct.robot.Arm
    G_POS: Target gripper position. Default: Current position + 0.02'''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else ct.robot.Arm
  g_pos= args[1] if len(args)>1 else ct.robot.GripperPos(arm)+0.02
  blocking= args[2] if len(args)>2 else False
  ct.Run('fv.ctrl_params')
  ct.Run('fv.grasp','off',arm)
  ct.Run('fv.hold','off',arm)
  ct.Run('fv.openif','off',arm)
  #ct.robot.OpenGripper(arm)
  ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], blocking=blocking)
  ct.Run('fv.fv','start_detect_obj',arm)


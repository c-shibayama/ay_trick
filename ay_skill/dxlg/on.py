#!/usr/bin/python
from core_tool import *
def Help():
  return '''Enable the torque a Dynamixel gripper.
  Usage: dxlg.on
  '''
def Run(ct,*args):
  ct.robot.EndEff().Stop(blocking=True)
  ct.robot.EndEff().Activate()

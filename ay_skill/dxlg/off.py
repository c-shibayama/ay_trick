#!/usr/bin/python
from core_tool import *
def Help():
  return '''Disable the torque a Dynamixel gripper.
  Usage: dxlg.off
  '''
def Run(ct,*args):
  ct.robot.EndEff().Deactivate()

#!/usr/bin/python
from core_tool import *
def Help():
  return '''Control gripper position.
  Usage:
    grip POSITION
      Move gripper to POSITION (floating-point value).
    grip 'open'
      Open gripper.
    grip 'close'
      Close gripper.
    grip
    grip 'pos'
      Get current position.
'''
def Run(ct,*args):
  if len(args)==0 or args[0]=='pos':  print ct.robot.GripperPos()
  elif args[0]=='open':  ct.robot.OpenGripper()
  elif args[0]=='close':  ct.robot.CloseGripper()
  else:  ct.robot.MoveGripper(float(args[0]))


#!/usr/bin/python
from core_tool import *
def Help():
  return '''Enable joints of Mikata Arm.
  Usage: mikata.on [JOINT]
    JOINT Joint to enable.
      'all': All joints.
      'g': Gripper joint. '''
def Run(ct,*args):
  joint= args[0] if len(args)>0 else 'all'

  if joint=='all':
    CPrint(0,'Caution: Robot torque is enabled.')
    if AskYesNo():
      ct.robot.mikata.EnableTorque()
    else:
      print 'Canceled.'
  elif joint=='g':
    joint= ct.robot.mikata.JointNames()[-1]
    ct.robot.mikata.EnableTorque([joint])

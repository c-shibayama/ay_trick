#!/usr/bin/python
from core_tool import *
def Help():
  return '''Disable joints of Mikata Arm.
  Usage: mikata.off [JOINT]
    JOINT Joint to disable.
      'all': All joints.
      'g': Gripper joint. '''
def Run(ct,*args):
  joint= args[0] if len(args)>0 else 'all'

  if joint=='all':
    CPrint(0,'Caution: Robot torque is disabled.')
    if AskYesNo():
      ct.robot.mikata.DisableTorque()
    else:
      print 'Canceled.'
  elif joint=='g':
    joint= ct.robot.mikata.JointNames()[-1]
    ct.robot.mikata.DisableTorque([joint])

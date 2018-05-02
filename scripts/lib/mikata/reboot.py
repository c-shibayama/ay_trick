#!/usr/bin/python
from core_tool import *
def Help():
  return '''Reboot joints of Mikata Arm.
  Usage: mikata.reboot [JOINT]
    JOINT Joint to reboot.
      'all': All joints.
      'g': Gripper joint. '''
def Run(ct,*args):
  joint= args[0] if len(args)>0 else 'all'

  if joint=='all':
    CPrint(0,'Caution: Robot torque is disabled for short time and then enabled.')
    if AskYesNo():
      ct.robot.mikata.Reboot()
      rospy.sleep(0.1)
      ct.robot.mikata.EnableTorque()
    else:
      print 'Canceled.'
  elif joint=='g':
    joint= ct.robot.mikata.JointNames()[-1]
    ct.robot.mikata.Reboot([joint])
    rospy.sleep(0.1)
    ct.robot.mikata.EnableTorque([joint])

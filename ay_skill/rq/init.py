#!/usr/bin/python
from core_tool import *
def Help():
  return '''Template of script.
  Usage: template'''
def Run(ct,*args):
  for i in range(ct.robot.NumArms):
    if ct.robot.EndEff(i).Is('Robotiq'):
      CPrint(1,'Initializing Robotiq gripper of {arm}-arm...'.format(arm=ct.robot.ArmStr(i)))
      if ct.robot.EndEff(i).Init():
        print 'OK'
        rospy.sleep(2.0)
        print 'Opening gripper...'
        ct.robot.EndEff(i).Open(blocking=True)
        print 'Done.'
      else:
        print 'Failed'

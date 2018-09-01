#!/usr/bin/python
from core_tool import *
def Help():
  return '''Configure Mikata Arm.
  Usage: mikata.config '''
def Run(ct,*args):
  all_joints= ct.robot.JointNames()+['gripper_joint_5']
  print 'RETURN_DELAY_TIME='
  print ct.robot.DxlRead('RETURN_DELAY_TIME',all_joints)
  print 'Configuring RETURN_DELAY_TIME'
  ct.robot.DxlWrite('RETURN_DELAY_TIME',{j:0 for j in all_joints})
  print 'RETURN_DELAY_TIME='
  print ct.robot.DxlRead('RETURN_DELAY_TIME',all_joints)

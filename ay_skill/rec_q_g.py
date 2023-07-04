#!/usr/bin/python
from core_tool import *
def Help():
  return '''Record joint (q) and gripper (g) positions using the joystick controller.
  Usage: rec_q_g [FILENAME]
    FILENAME: File to dump the record in YAML format (default: None).
        If None, the record is just printed.

  This script repeats the joystick control command (j) and then record
  the current positions of the joints and the gripper.
  Each cycle, there is a prompt to continue or quit.
  '''

def RecCurrent(ct, q_traj, g_traj):
  print 'Adding the current joint and gripper positions:', ct.robot.Q(), ct.robot.GripperPos()
  q_traj.append(ct.robot.Q())
  g_traj.append(ct.robot.GripperPos())

def Run(ct,*args):
  filename= args[0] if len(args)>0 else None
  q_traj= []
  g_traj= []
  #RecCurrent(ct, q_traj, g_traj)  #Record the initial positions.
  while not rospy.is_shutdown():
    print '---'
    print 'The joystick controller j is activated.'
    print 'Move the robot and the gripper with the joystick to positions to be recorded.'
    print 'Continue?'
    if not KBHAskYesNo():  break
    ct.Run('j')
    RecCurrent(ct, q_traj, g_traj)

  print 'Finished.'
  print '---'
  record= dict(q_traj=q_traj, g_traj=g_traj)
  print DumpYAML(record)
  if filename is not None:
    SaveYAML(record, filename)
    CPrint(1,'The record is saved into: {}'.format(filename))
  return q_traj, g_traj

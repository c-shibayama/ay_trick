#!/usr/bin/python
from core_tool import *
FollowGTraj= SmartImportReload('g_traj').FollowGTraj

def Help():
  return '''Play joint (q) and gripper (g) positions with intervals (dt) saved in a YAML file.
  Usage: play_q_g FILENAME [, INTERACTIVE]
    FILENAME: File in YAML format storing joint trajectory (q_traj), gripper trajectory (g_traj),
      and interval trajectory (dt_traj).
    INTERACTIVE: If True, there is a prompt to confirm to execute a motion (default:True).

  Example of FILENAME content:
    dt_traj:
      - 3.0
      - 2.0
      - 2.0
    g_traj:
      - 0.05
      - 0.04
      - 0.05
    q_traj:
      - [-0.5, -2.0, 2.3, -2.5, -1.0, 1.6]
      - [-0.2, -1.6, 2.1, -2.1, -1.5, 1.2]
      - [-0.5, -2.0, 2.3, -2.5, -1.0, 1.6]

  This script follows the positions on q_traj and g_traj one by one with the intervals dt_traj.
  '''

def Run(ct,*args):
  filename= args[0]
  interactive= args[1] if len(args)>1 else True
  data= LoadYAML(filename)
  q_traj,g_traj,dt_traj= data['q_traj'],data['g_traj'],data['dt_traj']
  assert(len(q_traj)==len(g_traj)==len(dt_traj))
  for q,g,dt in zip(q_traj,g_traj,dt_traj):
    print 'Move the robot and the gripper:'
    print '  q: {}'.format(q)
    print '  g: {}'.format(g)
    print '  dt: {}'.format(dt)
    if interactive:
      print 'Continue?'
      if not KBHAskYesNo():  break
    FollowGTraj(ct, [g], [dt], blocking=False)
    ct.robot.MoveToQ(q, dt=dt, blocking=True)
    ct.thread_manager.Join('follow_gripper_traj')

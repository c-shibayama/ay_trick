#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm to follow a target end-effector trajectory.
  Warning: Be careful to moving area of robot.
  Usage: ex.follow_x_traj1
  '''
def Run(ct,*args):
  arm= RIGHT
  x= list(ct.robot.FK(arm=arm))  #Current end-effector pose
  d= 0.05
  x_traj= [
      x,
      [x[0],x[1]+d,x[2]] + x[3:],
      [x[0],x[1],x[2]-d] + x[3:],
      [x[0],x[1]-d,x[2]] + x[3:],
      [x[0],x[1],x[2]+d] + x[3:],
      x
    ]
  t_traj= [0.0, 3.0, 6.0, 9.0, 12.0, 15.0]
  ct.robot.FollowXTraj(x_traj, t_traj, arm=arm)

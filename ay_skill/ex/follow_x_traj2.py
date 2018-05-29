#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm to follow a target end-effector trajectory.
  Warning: Be careful to moving area of robot.
  Usage: ex.follow_x_traj2
  '''
def Run(ct,*args):
  arm= RIGHT
  x= list(ct.robot.FK(arm=arm))  #Current end-effector pose
  d= 0.01
  x_traj= [
      x,
      [x[0]+d,x[1],x[2]] + x[3:],
      [x[0]-d,x[1],x[2]] + x[3:],
      #[x[0]+d,x[1],x[2]] + x[3:],
      #[x[0]-d,x[1],x[2]] + x[3:],
      [x[0],x[1],x[2]+d] + x[3:],
      [x[0],x[1],x[2]-d] + x[3:],
      #[x[0],x[1],x[2]+d] + x[3:],
      #[x[0],x[1],x[2]-d] + x[3:],
      [x[0],x[1]+d,x[2]] + x[3:],
      [x[0],x[1]-d,x[2]] + x[3:],
      #[x[0],x[1]+d,x[2]] + x[3:],
      #[x[0],x[1]-d,x[2]] + x[3:],
      x
    ]
  t_traj= [i*1.0 for i in range(len(x_traj))]
  ct.robot.FollowXTraj(x_traj, t_traj, arm=arm)

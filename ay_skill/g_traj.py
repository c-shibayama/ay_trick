#!/usr/bin/python
from core_tool import *
def Help():
  return '''Define functions of gripper trajectory control.
  Usage: Import functions from other script.
  e.g.
    FollowGTraj= SmartImportReload('g_traj').FollowGTraj
  '''
def Run(ct,*args):
  return Help()


#Follow a gripper trajectory (spline interpolation).
def FollowGTrajLoopSpline(th_info, ct, arm, g_traj, t_traj, callback):
  assert(len(g_traj)==len(t_traj))
  rate= 100.0
  rate_adjuster= rospy.Rate(rate)

  #If no initial point:
  if t_traj[0]>1.0e-3:
    g_traj= [ct.robot.GripperPos(arm=arm)]+list(g_traj)
    t_traj= [0.0]+list(t_traj)
  data_sp= [[t,g] for t,g in zip(t_traj,g_traj)]
  spline= TCubicHermiteSpline()
  spline.Initialize(data_sp, tan_method=spline.CARDINAL, c=1.0, m=0.0)
  #NOTE: Setting c to 1.0, spline does not overshoot.

  g_0= ct.robot.GripperPos(arm=arm)
  t0= rospy.Time.now()
  while th_info.IsRunning() and not rospy.is_shutdown():
    t_elapsed= (rospy.Time.now()-t0).to_sec()
    if t_elapsed<t_traj[-1]:
      g_trg= spline.Evaluate(t_elapsed,with_tan=False)
    else:
      g_trg= g_traj[-1]
    if callback is not None:
      index= spline.FindIdx(t_elapsed, spline.idx_prev)
      g_trg= callback(g_trg, t_elapsed, index)
    ct.robot.MoveGripper(g_trg,arm=arm,blocking=False)
    if t_elapsed>=t_traj[-1]:  break
    rate_adjuster.sleep()
  print 'Finishing FollowGTrajLoopSpline...'
  #ct.robot.MoveGripper(g_traj[-1],arm=arm,blocking=False)

#Follow a gripper trajectory (linear interpolation).
def FollowGTrajLoopLinear(th_info, ct, arm, g_traj, t_traj, callback):
  assert(len(g_traj)==len(t_traj))
  rate= 100.0
  rate_adjuster= rospy.Rate(rate)

  #If no initial point:
  if t_traj[0]>1.0e-3:
    g_traj= [ct.robot.GripperPos(arm=arm)]+list(g_traj)
    t_traj= [0.0]+list(t_traj)

  g_0= ct.robot.GripperPos(arm=arm)
  t0= rospy.Time.now()
  while th_info.IsRunning() and not rospy.is_shutdown():
    t_elapsed= (rospy.Time.now()-t0).to_sec()
    if t_elapsed<t_traj[-1]:
      index_next= next(i for i,t in enumerate(t_traj) if t>=t_elapsed)
      if index_next==0:  index_next+= 1
      index= index_next-1
      assert(index_next<len(t_traj))
      t_1,t_2= t_traj[index],t_traj[index_next]
      g_1,g_2= g_traj[index],g_traj[index_next]
      if t_2-t_1<1.0e-3:
        g_trg= g_2
      else:
        g_trg= g_1+(g_2-g_1)*(t_elapsed-t_1)/(t_2-t_1)
    else:
      index= len(t_traj)-1
      g_trg= g_traj[-1]
    if callback is not None:
      g_trg= callback(g_trg, t_elapsed, index)
    ct.robot.MoveGripper(g_trg,arm=arm,blocking=False)
    if t_elapsed>=t_traj[-1]:  break
    rate_adjuster.sleep()
  print 'Finishing FollowGTrajLoopLinear...'
  #ct.robot.MoveGripper(g_traj[-1],arm=arm,blocking=False)

'''Follow a gripper trajectory consisting of g_traj and t_traj.
  arm: arm id, or None (==currarm).
  g_traj: list of gripper targets.
  t_traj: list of times from start.
  blocking: False: move background, True: wait until motion ends.
  callback: function to modify the gripper target position.
  mode: Interpolation mode (options: spline, linear).
TODO:Implement this function as a method of ct.robot. '''
def FollowGTraj(ct, g_traj, t_traj, arm=None, blocking=False, callback=None, mode='linear'):
  if arm is None:  arm= ct.robot.Arm
  ct.thread_manager.Stop('follow_gripper_traj')
  if mode=='spline':
    ct.thread_manager.Add(name='follow_gripper_traj', target=lambda th_info: FollowGTrajLoopSpline(th_info,ct,arm,g_traj,t_traj,callback))
  elif mode=='linear':
    ct.thread_manager.Add(name='follow_gripper_traj', target=lambda th_info: FollowGTrajLoopLinear(th_info,ct,arm,g_traj,t_traj,callback))
  else:
    raise Exception('FollowGTraj: Unknown mode: {}'.format(mode))
  if blocking==True:
    ct.thread_manager.Join('follow_gripper_traj')

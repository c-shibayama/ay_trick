#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to a Cartesian pose with trajectory planning.
  Usage: move_to_x X_TRG, DURATION, X_EXT [, ARM [, SITUATION [, CONSERVATIVE]]]
    X_TRG: target pose
    DURATION: duration of motion
    X_EXT: task space in wrist frame
    ARM: hand id (default: ct.robot.Arm)
    SITUATION: extra parameters for adv.plan_traj (default: {})
    CONSERVATIVE: robot behaves conservatively (default: False)
  '''
def Run(ct,*args):
  x_trg= args[0]
  duration= args[1]
  x_ext= args[2]
  arm= args[3] if len(args)>3 else ct.robot.Arm
  situation= args[4] if len(args)>4 else {}
  conservative= args[5] if len(args)>5 else False

  #situation['objs']= [bottle,cup]
  situation['handid']= arm
  situation['l_x_ext']= x_ext
  situation['x_target']= x_trg
  situation['dt']= duration
  #situation['N']= 40
  #situation['bb_margin']= 1.0

  ct.DelAttr(TMP,'q_traj')
  ct.DelAttr(TMP,'t_traj')
  ct.DelAttr(TMP,'x_traj')
  ct.Run('adv.plan_traj', situation)

  if not ct.HasAttr(TMP,'q_traj') or not ct.HasAttr(TMP,'t_traj'):
    print 'Failed to infer q_traj/t_traj'
    return FailureCode('infer_q_traj')
  if conservative:
    print 'Follow the trajectory?'
    if not AskYesNo():  return FailureCode('canceled')

  q_traj= ct.GetAttr(TMP,'q_traj')
  t_traj= ct.GetAttr(TMP,'t_traj')
  #print 'q_traj',q_traj
  #print 't_traj',t_traj
  LimitQTrajVel(q_start=ct.robot.Q(arm), q_traj=q_traj, t_traj=t_traj, qvel_limits=ct.robot.JointVelLimits(arm))
  #print 'Modified q_traj',q_traj
  #print 'Modified t_traj',t_traj
  ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=True)

  return SUCCESS_CODE

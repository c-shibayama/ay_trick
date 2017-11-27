#!/usr/bin/python
from core_tool import *
def Help():
  return '''Velocity control tools for Motoman.  Do not use this directly.
    Usage:
      velctrl= ct.Load('moto.velctrl').TVelCtrl(ct)
      try:
        while ...:
          dq= [...]
          velctrl.Step(dq)
      finally:
        velctrl.Finish()
    '''
def Run(ct,*args):
  print 'Error:',Help()

class TVelCtrl(object):
  #ct: core_tool.
  #rate: Control time cycle in Hz.
  def __init__(self, ct, rate=40):
    self.rate= rate
    self.ct= ct
    self.arm= 0
    self.rate_adjuster= rospy.Rate(self.rate)
    self.dq_prev= [0.0]*7  #Hack for missing joint velocity.

  def Rate(self):
    return self.rate
  def TimeStep(self):
    return 1.0/float(self.rate)

  #dq: Joint angular velocity.
  #dq_lim: Limit of joint angular velocity.
  #q_limit_th: Threshold to detect if a joint angle is on the limit.
  #sleep: If True, this function waits until the end of the control cycle.
  def Step(self, dq, dq_lim=40.0, q_limit_th=0.02, sleep=True):
    ct= self.ct
    arm= self.arm
    dq= copy.deepcopy(dq)
    dt= self.TimeStep()
    #print ' '.join(map(lambda f:'%0.2f'%f,dq))

    dq_max= max(map(abs,dq))
    if dq_max>dq_lim:  dq= [v*(dq_lim/dq_max) for v in dq]

    q= ct.robot.Q(arm=arm)
    q2= [0.0]*7
    for j,(qi,dqi,qmini,qmaxi) in enumerate(zip(q,dq,*ct.robot.JointLimits(arm))):
      q2[j]= qi+dt*dqi
      if q2[j]<qmini+q_limit_th:
        q2[j]= qmini+q_limit_th
        dq[j]= (q2[j]-qi)/dt
      if q2[j]>qmaxi-q_limit_th:
        q2[j]= qmaxi-q_limit_th
        dq[j]= (q2[j]-qi)/dt

    t_traj= [0.0, dt]
    q_traj= [ct.robot.Q(arm=arm), q2]
    dq_traj= [ct.robot.DQ(arm=arm), dq]
    if len(dq_traj[0])==0:  dq_traj[0]= self.dq_prev
    traj= ToROSTrajectory(ct.robot.JointNames(arm), q_traj, t_traj, dq_traj)
    with ct.robot.control_locker:
      ct.robot.pub.joint_path_command.publish(traj)
    self.dq_prev= dq

    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    ct= self.ct
    arm= self.arm
    dt= self.TimeStep()
    t_traj= [0.0, dt]
    q_traj= [ct.robot.Q(arm=arm), ct.robot.Q(arm=arm)]
    dq_traj= [ct.robot.DQ(arm=arm), [0.0]*7]
    if len(dq_traj[0])==0:  dq_traj[0]= self.dq_prev
    traj= ToROSTrajectory(ct.robot.JointNames(arm), q_traj, t_traj, dq_traj)
    with ct.robot.control_locker:
      ct.robot.pub.joint_path_command.publish(traj)
    self.dq_prev= [0.0]*7

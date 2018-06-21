#!/usr/bin/python
from core_tool import *
def Help():
  return '''Velocity control tools for UR.  Do not use this directly.
    Usage:
      velctrl= ct.Load('ur.velctrl').TVelCtrl(ct,arm=LEFT)
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
  def __init__(self, ct, arm, rate=125):
    self.rate= rate
    self.ct= ct
    self.arm= arm
    ct.AddPub('trg_vel', '/ur_driver/joint_speed', trajectory_msgs.msg.JointTrajectory)
    self.rate_adjuster= rospy.Rate(self.rate)
    self.traj= trajectory_msgs.msg.JointTrajectory()
    self.traj.joint_names= ct.robot.JointNames(arm)
    self.traj.points= [trajectory_msgs.msg.JointTrajectoryPoint()]

  def Rate(self):
    return self.rate
  def TimeStep(self):
    return 1.0/float(self.rate)

  #dq: Joint angular velocity (target; rad/s).
  #dq_lim: Limit of joint angular velocity (rad/s).
  #ddq_lim: Limit of joint angular acceleration (rad/s**2).
  #q_limit_th: Threshold to detect if a joint angle is on the limit.
  #sleep: If True, this function waits until the end of the control cycle.
  def Step(self, dq, dq_lim=40.0, ddq_lim=25.0, q_limit_th=0.02, sleep=True):
    ct= self.ct
    arm= self.arm
    dq= copy.deepcopy(dq)
    dt= self.TimeStep()
    #print ' '.join(map(lambda f:'%0.2f'%f,dq))

    #Get current state:
    q= ct.robot.Q(arm=arm)
    dq0= ct.robot.DQ(arm=arm)

    #Limit accelerations:
    ddq_max= max([abs(v-v0)/dt for v,v0 in zip(dq,dq0)])
    if ddq_max>ddq_lim:
      scale= ddq_lim/ddq_max
      #print dq, dq0, [(v-v0)/dt for v,v0 in zip(dq,dq0)], scale
      dq= [scale*v+(1.0-scale)*v0 for v,v0 in zip(dq,dq0)]
      CPrint(3,'ur.velctrl: dq is modified due to exceeding ddq_lim;',ddq_max,ddq_lim)
      #print dq
    #Limit velocities:
    dq_max= max(map(abs,dq))
    if dq_max>dq_lim:
      scale= dq_lim/dq_max
      dq= [v*scale for v in dq]
      #CPrint(3,'ur.velctrl: dq is modified due to exceeding dq_lim;',dq_max,dq_lim)

    q2= [0.0]*7
    for j,(qi,dqi,qmini,qmaxi) in enumerate(zip(q,dq,*ct.robot.JointLimits(arm))):
      q2[j]= qi+dt*dqi
      if q2[j]<qmini+q_limit_th:
        q2[j]= qmini+q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'ur.velctrl: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)
      if q2[j]>qmaxi-q_limit_th:
        q2[j]= qmaxi-q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'ur.velctrl: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)

    self.traj.points[0].velocities= dq
    self.traj.points[0].accelerations= [10.0]*6

    ct.pub.trg_vel.publish(self.traj)
    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    ct= self.ct
    self.traj.points[0].velocities= [0.0]*ct.robot.DoF(self.arm)
    ct.pub.trg_vel.publish(self.traj)

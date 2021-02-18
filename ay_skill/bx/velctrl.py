#!/usr/bin/python
from core_tool import *
import std_msgs.msg
def Help():
  return '''Velocity control tools for Baxter.  Do not use this directly.
    Usage:
      velctrl= ct.Load('bx.velctrl').TVelCtrl(arm,ct)
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
  __metaclass__= TMultiSingleton

  #ct: core_tool.
  #rate: Control time cycle in Hz.
  def __init__(self, arm, ct, rate=None):
    self.rate= rate if rate is not None else 500
    self.ct= ct
    self.arm= arm
    ct.AddPub('js_rate', 'robot/joint_state_publish_rate', std_msgs.msg.UInt16)
    ct.pub.js_rate.publish(self.rate)
    self.rate_adjuster= rospy.Rate(self.rate)

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

    ct.pub.js_rate.publish(self.rate)

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

    cmd= {joint: dq[j] for j,joint in enumerate(ct.robot.JointNames(arm))}
    ct.robot.limbs[arm].set_joint_velocities(cmd)
    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    self.__class__.Delete(self.arm)
    if self.__class__.NumReferences(self.arm)>0:  return
    ct= self.ct
    arm= self.arm
    cmd= {joint: 0.0 for j,joint in enumerate(ct.robot.JointNames(arm))}
    ct.robot.limbs[arm].set_joint_velocities(cmd)
    ct.robot.limbs[arm].exit_control_mode()
    ct.pub.js_rate.publish(100)  #100 Hz is default joint state rate

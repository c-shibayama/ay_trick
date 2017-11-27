#!/usr/bin/python
from core_tool import *
def Help():
  return '''Torque control tools for Baxter.  Do not use this directly.
    Usage:
      tqctrl= ct.Load('bx.tqctrl').TTqCtrl(ct,arm=LEFT)
      try:
        while ...:
          tq= [...]
          tqctrl.Step(tq)
      finally:
        tqctrl.Finish()
    '''
def Run(ct,*args):
  print 'Error:',Help()

class TTqCtrl(object):
  #ct: core_tool.
  #rate: Control time cycle in Hz.
  def __init__(self, ct, arm, rate=500):
    self.rate= rate
    self.ct= ct
    self.arm= arm
    ct.AddPub('js_rate', 'robot/joint_state_publish_rate', std_msgs.msg.UInt16)
    ct.pub.js_rate.publish(self.rate)
    self.rate_adjuster= rospy.Rate(self.rate)

  def Rate(self):
    return self.rate
  def TimeStep(self):
    return 1.0/float(self.rate)

  #tq: Joint torque.
  #tq_lim: Limit of joint torque.
  #q_limit_th: Threshold to detect if a joint angle is on the limit.
  #sleep: If True, this function waits until the end of the control cycle.
  def Step(self, tq, tq_lim=40.0, q_limit_th=0.02, sleep=True):
    ct= self.ct
    arm= self.arm
    tq= copy.deepcopy(tq)
    #print ' '.join(map(lambda f:'%0.2f'%f,tq))

    ct.pub.js_rate.publish(self.rate)

    q= ct.robot.Q(arm=arm)
    for j,(qi,qmini,qmaxi) in enumerate(zip(q,*ct.robot.JointLimits(arm))):
      if qi<qmini+q_limit_th or qi>qmaxi-q_limit_th:  tq[j]= 0.0

    tq_max= max(map(abs,tq))
    if tq_max>tq_lim:  tq= [v*(tq_lim/tq_max) for v in tq]

    cmd= {joint: tq[j] for j,joint in enumerate(ct.robot.JointNames(arm))}
    ct.robot.limbs[arm].set_joint_torques(cmd)
    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    ct= self.ct
    arm= self.arm
    ct.robot.limbs[arm].exit_control_mode()
    ct.pub.js_rate.publish(100)  #100 Hz is default joint state rate

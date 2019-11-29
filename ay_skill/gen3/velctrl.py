#!/usr/bin/python
from core_tool import *
import kortex_driver.msg
import kortex_driver.srv
def Help():
  return '''Velocity control tools for Gen3.  Do not use this directly.
    Usage:
      velctrl= ct.Load('gen3.velctrl').TVelCtrl(arm,ct)
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
  #dq_lim: Limit of joint angular velocity (rad/s).
  #ddq_lim: Limit of joint angular acceleration (rad/s**2).
  #q_limit_th: Threshold to detect if a joint angle is on the limit.
  def __init__(self, arm, ct, rate=None, dq_lim=40.0, ddq_lim=20.0, q_limit_th=0.02):
    #ddq_lim can be: lambda dq:3.0 if max(map(abs,dq))>0.1 else 1.0
    if rate is None:
      ns= ct.robot.NS()
      rate= rospy.get_param('/'+ns+'/'+ns+'_driver/cyclic_data_publish_rate', None)
    self.rate= rate
    self.dq_lim= dq_lim
    self.ddq_lim= ddq_lim
    self.q_limit_th= q_limit_th
    self.ct= ct
    self.arm= arm

    ct.AddSrvP('joint_speeds', '/gen3a/base/send_joint_speeds_command', kortex_driver.srv.SendJointSpeedsCommand, persistent=False, time_out=3.0)

    self.rate_adjuster= rospy.Rate(self.rate)
    self.last_dq= [0.0]*ct.robot.DoF(self.arm)

    speed_req= kortex_driver.srv.SendJointSpeedsCommandRequest()
    #NOTE: JointSpeed/value is in DEGREES per second.
    #cf. https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/JointSpeed.md
    for jidx,jname in enumerate(ct.robot.JointNames(self.arm)):
      joint_speed= kortex_driver.msg.JointSpeed()
      joint_speed.joint_identifier= jidx
      joint_speed.value= 0.0
      speed_req.input.joint_speeds.append(joint_speed)
    self.speed_req= speed_req

  #Convert a joint velocity vector to a service request.
  def SetSpeedReq(self, dq):
    for joint_speed,dq_d in zip(self.speed_req.input.joint_speeds,dq):
      #NOTE: JointSpeed/value is in DEGREES per second.
      joint_speed.value= RadToDeg(dq_d)

  def Rate(self):
    return self.rate
  def TimeStep(self):
    return 1.0/float(self.rate)

  #dq: Joint angular velocity (target; rad/s).
  #sleep: If True, this function waits until the end of the control cycle.
  def Step(self, dq, sleep=True):
    ct= self.ct
    arm= self.arm
    dof= ct.robot.DoF(arm)
    #dq= copy.deepcopy(dq)
    dq= [v if abs(v)<vmax else v/vmax for v,vmax in zip(dq,ct.robot.JointVelLimits(arm))]
    dt= self.TimeStep()
    #print ' '.join(map(lambda f:'%0.2f'%f,dq))

    #if self.rate_adjuster.remaining().to_sec()<-self.velctrl.dt:
      #CPrint(4,'Loosing real-time control(0):', self.rate_adjuster.remaining().to_sec())
      ##self.last_dq= ct.robot.DQ(arm=arm)
      #self.last_dq= [0.0]*dof

    #Get current state:
    q= ct.robot.Q(arm=arm)
    #dq0= ct.robot.DQ(arm=arm)
    dq0= self.last_dq
    '''NOTE
    We use last-dq (target velocities) rather than current actual velocities
    in order to avoid oscillation.
    '''

    #Limit accelerations:
    ddq_max= max([abs(v-v0)/dt for v,v0 in zip(dq,dq0)])
    ddq_lim= self.ddq_lim if not callable(self.ddq_lim) else self.ddq_lim(dq0)
    if ddq_max>ddq_lim:
      scale= ddq_lim/ddq_max
      #print dq, dq0, [(v-v0)/dt for v,v0 in zip(dq,dq0)], scale
      dq= [scale*v+(1.0-scale)*v0 for v,v0 in zip(dq,dq0)]
      CPrint(3,'gen3.velctrl: dq is modified due to exceeding ddq_lim;',ddq_max,ddq_lim)
      #print scale,dq0,dq
    #Limit velocities:
    dq_max= max(map(abs,dq))
    if dq_max>self.dq_lim:
      scale= self.dq_lim/dq_max
      dq= [v*scale for v in dq]
      CPrint(3,'gen3.velctrl: dq is modified due to exceeding dq_lim;',dq_max,self.dq_lim)

    q2= [0.0]*dof
    for j,(qi,dqi,qmini,qmaxi) in enumerate(zip(q,dq,*ct.robot.JointLimits(arm))):
      q2[j]= qi+dt*dqi
      if q2[j]<qmini+self.q_limit_th:
        q2[j]= qmini+self.q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'ur.velctrl: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)
      if q2[j]>qmaxi-self.q_limit_th:
        q2[j]= qmaxi-self.q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'ur.velctrl: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)

    #if self.rate_adjuster.remaining().to_sec()<0:
      #CPrint(4,'Loosing real-time control(1):', self.rate_adjuster.remaining().to_sec())
    self.SetSpeedReq(dq)
    ct.srvp['joint_speeds'](self.speed_req)
    self.last_dq= dq
    #print 'ur.velctrl:rate_adjuster.remaining:',self.rate_adjuster.remaining().to_sec()
    #if self.rate_adjuster.remaining().to_sec()<-self.velctrl.dt:
      #CPrint(4,'Loosing real-time control(2):', self.rate_adjuster.remaining().to_sec())
      ##self.last_dq= ct.robot.DQ(arm=arm)
      #self.last_dq= [0.0]*dof
    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    self.__class__.Delete(self.arm)
    if self.__class__.NumReferences(self.arm)>0:  return
    self.ct.DelSrvP('joint_speeds')

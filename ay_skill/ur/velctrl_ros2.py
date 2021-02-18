#!/usr/bin/python
from core_tool import *
import std_msgs.msg
import controller_manager_msgs.srv
def Help():
  return '''Velocity control tools for UR (uising the Universal_Robots_ROS_Driver package).  Do not use this directly.
  This code uses the ROS topic /joint_group_vel_controller/command.
    Usage:
      velctrl= ct.Load('ur.velctrl_ros2').TVelCtrl(arm,ct)
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
  def __init__(self, arm, ct, rate=None, dq_lim=40.0, ddq_lim=3.0, q_limit_th=0.02):
    self.rate= rate if rate is not None else 125
    self.dq_lim= dq_lim
    self.ddq_lim= ddq_lim
    self.q_limit_th= q_limit_th
    self.ct= ct
    self.arm= arm
    self.rate_adjuster= rospy.Rate(self.rate)
    self.last_dq= [0.0]*self.ct.robot.DoF(self.arm)

    self.msg= std_msgs.msg.Float64MultiArray()
    self.msg.data= [0.0]*self.ct.robot.DoF(self.arm)
    self.msg.layout.data_offset= 1
    self.mode= 'traj'

    self.StartVelCtrlMode()

  def Rate(self):
    return self.rate
  def TimeStep(self):
    return 1.0/float(self.rate)

  #dq: Joint angular velocity (target; rad/s).
  #sleep: If True, this function waits until the end of the control cycle.
  def Step(self, dq, sleep=True):
    if self.mode!='vel':
      print 'ur.velctrl: Warning: The velocity control mode is not active.'
      return

    ct= self.ct
    arm= self.arm
    dof= ct.robot.DoF(arm)
    dq= copy.deepcopy(dq)
    dt= self.TimeStep()
    #print ' '.join(map(lambda f:'%0.2f'%f,dq))

    #Get current state:
    q= ct.robot.Q(arm=arm)
    #dq0= ct.robot.DQ(arm=arm)
    dq0= self.last_dq
    '''NOTE
    We use last dq (target velocities) rather than current actual velocities
    in order to avoid oscillation.
    '''

    if not ct.robot.IsNormal():
      #self.Finish()  #NOTE: Do not call self.Finish since it deletes the singleton instance.
      self.StopVelCtrlMode()
      raise Exception('TVelCtrl has stopped as the robot is not normal state.')

    #Limit accelerations:
    ddq_max= max([abs(v-v0)/dt for v,v0 in zip(dq,dq0)])
    if ddq_max>self.ddq_lim:
      scale= self.ddq_lim/ddq_max
      #print dq, dq0, [(v-v0)/dt for v,v0 in zip(dq,dq0)], scale
      dq= [scale*v+(1.0-scale)*v0 for v,v0 in zip(dq,dq0)]
      CPrint(3,'ur.velctrl: dq is modified due to exceeding ddq_lim;',ddq_max,self.ddq_lim)
      #print scale,dq0,dqtmp,dq
    #Limit velocities:
    dq_max= max(map(abs,dq))
    if dq_max>self.dq_lim:
      scale= self.dq_lim/dq_max
      dq= [v*scale for v in dq]
      CPrint(3,'ur.velctrl: dq is modified due to exceeding dq_lim;',dq_max,self.dq_lim)

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

    self.msg.data= dq
    ct.robot.pub.joint_vel.publish(self.msg)
    self.last_dq= dq
    #print self.rate_adjuster.remaining().to_sec()
    if self.rate_adjuster.remaining().to_sec()<0:
      CPrint(4,'Loosing real-time control:', self.rate_adjuster.remaining().to_sec())
    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    self.__class__.Delete(self.arm)
    if self.__class__.NumReferences(self.arm)>0:  return
    self.StopVelCtrlMode()

  def StartVelCtrlMode(self):
    if self.mode=='vel':  return
    #Switch the control mode (traj --> vel):
    sw_ctrl_req= controller_manager_msgs.srv.SwitchControllerRequest()
    sw_ctrl_req.strictness= sw_ctrl_req.STRICT
    sw_ctrl_req.stop_controllers= ['scaled_pos_joint_traj_controller']
    sw_ctrl_req.start_controllers= []
    self.ct.robot.srvp.sw_ctrl(sw_ctrl_req)
    sw_ctrl_req.stop_controllers= []
    sw_ctrl_req.start_controllers= ['joint_group_vel_controller']
    self.ct.robot.srvp.sw_ctrl(sw_ctrl_req)
    self.mode= 'vel'

  def StopVelCtrlMode(self):
    if self.mode=='traj':  return
    #Make sure to stop the robot:
    #self.Step([0.0]*self.ct.robot.DoF(self.arm))  #NOTE: Do not run Step since it does not work in non normal state.
    self.msg.data= [0.0]*self.ct.robot.DoF(self.arm)
    self.ct.robot.pub.joint_vel.publish(self.msg)
    #Switch the control mode (vel --> traj):
    sw_ctrl_req= controller_manager_msgs.srv.SwitchControllerRequest()
    sw_ctrl_req.strictness= sw_ctrl_req.STRICT
    sw_ctrl_req.stop_controllers= ['joint_group_vel_controller']
    sw_ctrl_req.start_controllers= []
    self.ct.robot.srvp.sw_ctrl(sw_ctrl_req)
    sw_ctrl_req.stop_controllers= []
    sw_ctrl_req.start_controllers= ['scaled_pos_joint_traj_controller']
    self.ct.robot.srvp.sw_ctrl(sw_ctrl_req)
    self.mode= 'traj'

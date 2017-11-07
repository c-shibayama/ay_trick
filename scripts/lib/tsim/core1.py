#!/usr/bin/python
from core_tool import *
import std_msgs.msg
import std_srvs.srv
roslib.load_manifest('ay_sim_msgs')
import ay_sim_msgs.msg
import ay_sim_msgs.srv
def Help():
  return '''Core of ODE grasping and pouring simulation.
  Usage: do not call this directly.'''

def SetupServiceProxy(ct,l):
  if 'ode_get_config' not in ct.srvp:
    print 'Waiting for /ode_grpour_sim/get_config...'
    rospy.wait_for_service('/ode_grpour_sim/get_config',3.0)
    ct.srvp.ode_get_config= rospy.ServiceProxy('/ode_grpour_sim/get_config', ay_sim_msgs.srv.ODEGetConfig, persistent=False)
  if 'ode_reset2' not in ct.srvp:
    print 'Waiting for /ode_grpour_sim/reset2...'
    rospy.wait_for_service('/ode_grpour_sim/reset2',3.0)
    ct.srvp.ode_reset2= rospy.ServiceProxy('/ode_grpour_sim/reset2', ay_sim_msgs.srv.ODEReset2, persistent=False)
  if 'ode_pause' not in ct.srvp:
    print 'Waiting for /ode_grpour_sim/pause...'
    rospy.wait_for_service('/ode_grpour_sim/pause',3.0)
    ct.srvp.ode_pause= rospy.ServiceProxy('/ode_grpour_sim/pause', std_srvs.srv.Empty, persistent=False)
  if 'ode_resume' not in ct.srvp:
    print 'Waiting for /ode_grpour_sim/resume...'
    rospy.wait_for_service('/ode_grpour_sim/resume',3.0)
    ct.srvp.ode_resume= rospy.ServiceProxy('/ode_grpour_sim/resume', std_srvs.srv.Empty, persistent=False)

def SetupPubSub(ct,l):
  StopPubSub(ct,l)
  if 'ode_ppour' not in ct.pub:
    ct.pub.ode_ppour= rospy.Publisher("/ode_grpour_sim/ppour", std_msgs.msg.Float64MultiArray)
  if 'ode_theta' not in ct.pub:
    ct.pub.ode_theta= rospy.Publisher("/ode_grpour_sim/theta", std_msgs.msg.Float64)
  if 'ode_viz' not in ct.pub:
    ct.pub.ode_viz= rospy.Publisher("/ode_grpour_sim/viz", ay_sim_msgs.msg.ODEViz)
  if 'ode_sensors' not in ct.sub:
    ct.sub.ode_sensors= rospy.Subscriber("/ode_grpour_sim/sensors", ay_sim_msgs.msg.ODESensor, lambda msg:ODESensorCallback(msg,ct,l))
  if 'sensor_callback' not in l:
    l.sensor_callback= None

def StopPubSub(ct,l):
  if 'ode_sensors' in ct.sub:
    ct.sub.ode_sensors.unregister()
    del ct.sub.ode_sensors

def ODESensorCallback(msg,ct,l):
  l.sensors= msg
  #print l.sensors.num_src, l.sensors.num_rcv, l.sensors.num_flow, l.sensors.num_spill, l.sensors.num_bounce
  if l.sensor_callback!=None:
    l.sensor_callback()

def GetConfig(ct):
  return ct.srvp.ode_get_config().config

def ResetConfig(ct,config):
  ct.pub.ode_viz.publish(ay_sim_msgs.msg.ODEViz())  #Clear visualization
  req= ay_sim_msgs.srv.ODEReset2Request()
  req.config= config
  ct.srvp.ode_reset2(req)

def SimSleep(ct,l,dt):
  tc0= l.sensors.time
  while l.sensors.time-tc0<dt:
    time.sleep(dt*0.02)

def MoveDPPour(ct,l,dp):
  dt= l.config.TimeStep
  p_pour0= l.sensors.p_pour
  p_pour_msg= std_msgs.msg.Float64MultiArray()
  p_pour_msg.data= Vec(p_pour0) + Vec(dp)
  ct.pub.ode_ppour.publish(p_pour_msg)
  SimSleep(ct,l,dt)

def MoveDTheta(ct,l,dth):
  dt= l.config.TimeStep
  theta0= l.sensors.theta
  theta_msg= std_msgs.msg.Float64()
  theta_msg.data= theta0 + dth
  ct.pub.ode_theta.publish(theta_msg)
  SimSleep(ct,l,dt)

def MoveDPPourDTheta(ct,l,dp,dth):
  dt= l.config.TimeStep
  p_pour0= l.sensors.p_pour
  p_pour_msg= std_msgs.msg.Float64MultiArray()
  p_pour_msg.data= Vec(p_pour0) + Vec(dp)
  ct.pub.ode_ppour.publish(p_pour_msg)
  theta0= l.sensors.theta
  theta_msg= std_msgs.msg.Float64()
  theta_msg.data= theta0 + dth
  ct.pub.ode_theta.publish(theta_msg)
  SimSleep(ct,l,dt)

def MoveToTrgPPour(ct,l,p_pour_trg,spd):
  dt= l.config.TimeStep
  p_pour0= l.sensors.p_pour
  diff= Vec(p_pour_trg) - Vec(p_pour0)
  diff_norm= la.norm(diff)
  p_pour_msg= std_msgs.msg.Float64MultiArray()
  if diff_norm>spd*dt:
    p_pour_msg.data= Vec(p_pour0) + diff*(spd*dt)/diff_norm
    reached= False
  else:
    p_pour_msg.data= Vec(p_pour_trg)
    reached= True
  ct.pub.ode_ppour.publish(p_pour_msg)
  SimSleep(ct,l,dt)
  return reached

class TPause:
  def __init__(self, ct):
    self.ct= ct
  def __enter__(self, *args, **kwargs):
    self.ct.srvp.ode_pause()
    return self
  def __exit__(self, *args, **kwargs):
    self.ct.srvp.ode_resume()

def Run(ct,*args):
  print Help()

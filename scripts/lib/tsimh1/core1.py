#!/usr/bin/python
from core_tool import *
import std_msgs.msg
import std_srvs.srv
roslib.load_manifest('ay_sim_msgs')
import ay_sim_msgs.msg
import ay_sim_msgs.srv
def Help():
  return '''Core of ODE hopping robot simulation.
  Usage: do not call this directly.'''

def SetupServiceProxy(ct,l):
  ct.AddSrvP('ode_get_config', '/ode_hopper1/get_config',
            ay_sim_msgs.srv.ODEGetConfigHp1, persistent=False, time_out=3.0)
  ct.AddSrvP('ode_reset2', '/ode_hopper1/reset2',
            ay_sim_msgs.srv.ODESetConfigHp1, persistent=False, time_out=3.0)
  ct.AddSrvP('ode_pause', '/ode_hopper1/pause',
            std_srvs.srv.Empty, persistent=False, time_out=3.0)
  ct.AddSrvP('ode_resume', '/ode_hopper1/resume',
            std_srvs.srv.Empty, persistent=False, time_out=3.0)

def SetupPubSub(ct,l):
  StopPubSub(ct,l)
  ct.AddPub('ode_ctrl', '/ode_hopper1/control', ay_sim_msgs.msg.ODEControlHp1)
  ct.AddPub('ode_viz', '/ode_hopper1/viz', ay_sim_msgs.msg.ODEViz)
  ct.AddSub('ode_sensors', '/ode_hopper1/sensors',
           ay_sim_msgs.msg.ODESensorHp1,
           lambda msg:ODESensorCallback(msg,ct,l))
  if 'sensor_callback' not in l:
    l.sensor_callback= None

def StopPubSub(ct,l):
  ct.DelSub('ode_sensors')

def ODESensorCallback(msg,ct,l):
  l.sensors= msg
  #print l.sensors.Time
  if l.sensor_callback!=None:
    l.sensor_callback()

def GetConfig(ct):
  return ct.srvp.ode_get_config().config

def ResetConfig(ct,config):
  ct.pub.ode_viz.publish(ay_sim_msgs.msg.ODEViz())  #Clear visualization
  req= ay_sim_msgs.srv.ODESetConfigHp1Request()
  req.config= config
  ct.srvp.ode_reset2(req)

def SimSleep(ct,l,dt):
  tc0= l.sensors.Time
  while l.sensors.Time-tc0<dt:
    time.sleep(dt*0.02)

def MoveDTheta(ct,l,dth):
  dt= l.config.TimeStep
  theta0= Vec(l.sensors.JointAngles)
  ctrl_msg= ay_sim_msgs.msg.ODEControlHp1()
  ctrl_msg.Angles= theta0 + dth
  ct.pub.ode_ctrl.publish(ctrl_msg)
  SimSleep(ct,l,dt)

def TrackTraj(ct,l,q_traj,t_traj):
  assert(len(q_traj)==len(t_traj))
  t_prev= 0.0
  ctrl_msg= ay_sim_msgs.msg.ODEControlHp1()
  for q_curr,t_curr in zip(q_traj,t_traj):
    ctrl_msg.Angles= q_curr
    ct.pub.ode_ctrl.publish(ctrl_msg)
    SimSleep(ct,l,t_curr-t_prev)
    t_prev= t_curr

def Run(ct,*args):
  print Help()

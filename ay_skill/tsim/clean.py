#!/usr/bin/python
from core_tool import *
def Help():
  return '''Clean the ROS connections used in tsim.
  Usage: tsim.clean'''
def Run(ct,*args):
  ct.DelSub('ode_sensors')

  ct.DelPub('ode_ppour')
  ct.DelPub('ode_theta')
  ct.DelPub('ode_viz')

  ct.DelSrvP('ode_get_config')
  ct.DelSrvP('ode_reset2')
  ct.DelSrvP('ode_pause')
  ct.DelSrvP('ode_resume')

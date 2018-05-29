#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of test_spline for ODE grasping and pouring simulation (ver.2.1).
    Based on tsim.sm4
  Usage: tsim2.test_splinex'''

#import tsim.sm1
#reload(tsim.sm1)
#from tsim.sm1 import (
  #SetMaterial,
  #ResetFilter, ApplyFilter,
  #)

def TestConfigCallback(ct,l,sim):
  #l.config.RcvPos= [0.6, l.config.RcvPos[1], l.config.RcvPos[2]]
  l.config.RcvPos= [0.8, l.config.RcvPos[1], l.config.RcvPos[2]]
  #l.config.RcvPos= [1.2, l.config.RcvPos[1], l.config.RcvPos[2]]
  #l.config.RcvPos= [0.8+0.6*(random.random()-0.5), l.config.RcvPos[1], l.config.RcvPos[2]]
  CPrint(3,'l.config.RcvPos=',l.config.RcvPos)
  #l.config.ContactBounce= 0.7
  #l.config.ContactBounce= 0.1+0.8*(random.random())
  #CPrint(3,'l.config.ContactBounce=',l.config.ContactBounce)

def Run(ct,*args):
  l= TContainer(debug=True)
  l.config_callback= TestConfigCallback
  ct.Run('tsim2.setup', l)
  sim= ct.sim
  l= ct.sim_local

  try:
    ct.Run('tsim2.act.grab', {'gh_ratio':0.5})
    ct.Run('tsim2.act.move_to_rcv', {'p_pour_trg0':[l.sensors.x_rcv.position.x-0.1, l.sensors.x_rcv.position.z+1.0]})
    ct.Run('tsim2.act.move_to_pour', {'p_pour_trg':[l.sensors.x_rcv.position.x-0.1, l.sensors.x_rcv.position.z+1.0]})

    ct.Run('tsim2.test_spline', {})

  finally:
    sim.StopPubSub(ct,l)
    l.sensor_callback= None
    ct.srvp.ode_pause()

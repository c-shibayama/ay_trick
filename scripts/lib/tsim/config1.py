#!/usr/bin/python
from core_tool import *
def Help():
  return '''Change config of ODE grasping and pouring simulation.
  Usage: config1'''
def Run(ct,*args):
  l= TContainer(debug=True)
  sim= ct.Load('tsim.core1')

  sim.SetupServiceProxy(ct,l)
  sim.SetupPubSub(ct,l)

  ct.srvp.ode_resume()
  l.config= sim.GetConfig(ct)
  print 'Current config:',l.config

  #Setup config
  l.config.MaxContacts= 2
  l.config.TimeStep= 0.025
  l.config.Gravity= -1.0
  l.config.BallType= 0  #Sphere particles
  #l.config.BallType= 1  #Box particles
  l.config.SrcSize2H= 0.03  #Mouth size; Default: 0.03
  #Bounce balls:
  l.config.ContactBounce= 0.7
  l.config.ContactBounceVel= 0.2
  l.config.ViscosityParam1= 0.0  #Default: 0.0
  #Natto:
  #l.config.ContactBounce= 0.1
  #l.config.ContactBounceVel= 0.01
  #l.config.ViscosityParam1= 1.5e-6  #Default: 0.0
  #l.config.ViscosityMaxDist= 0.1  #Default: 0.1
  #Ketchup:
  #l.config.ContactBounce= 0.1
  #l.config.ContactBounceVel= 0.01
  #l.config.ViscosityParam1= 2.5e-7  #Default: 0.0
  #l.config.ViscosityMaxDist= 0.2  #Default: 0.1
  #Slime???:
  #l.config.ContactBounce= 0.1
  #l.config.ContactBounceVel= 0.01
  #l.config.ViscosityParam1= 1.0e-5  #Default: 0.0
  #l.config.ViscosityMaxDist= 0.1  #Default: 0.1

  #Reset to get state for plan
  sim.ResetConfig(ct,l.config)
  time.sleep(0.1)  #Wait for l.sensors is updated
  ct.srvp.ode_pause()  #Pause to wait grasp plan
  print '---------------'
  print 'New config:',sim.GetConfig(ct)
  print '---------------'
  print 'Run tsim.clean to disconnect'

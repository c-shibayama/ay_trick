#!/usr/bin/python
from core_tool import *
def Help():
  return '''Template of script.
  Usage: template'''

def TestConfigCallback(ct,l,sim):
  l.config.RcvPos= [0.8+0.6*(random.random()-0.5), l.config.RcvPos[1], l.config.RcvPos[2]]
  CPrint(3,'l.config.RcvPos=',l.config.RcvPos)

  rsx= Rand(0.25,0.5)
  rsy= Rand(0.1,0.2)/rsx
  rsz= Rand(0.2,0.5)
  l.config.RcvSize= [rsx, rsy, rsz]

  l.config.SrcSize2H= Rand(0.02,0.09)  #Mouth size of source container
  CPrint(3,'l.config.ViscosityParam1=',l.config.ViscosityParam1)
  CPrint(3,'l.config.SrcSize2H=',l.config.SrcSize2H)

def Run(ct,*args):
  l= TContainer(debug=True)
  l.config_callback= TestConfigCallback
  ct.Run('tsim2.setup', l)
  sim= ct.sim
  l= ct.sim_local
  ct.srvp.ode_pause()

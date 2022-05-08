#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to a parking pose.
  Usage: mikata.move_to_park [DT [, BLOCKING]]'''
def Run(ct,*args):
  dt= args[0] if len(args)>0 else 4.0
  blocking= args[1] if len(args)>1 else True
  q_trg= [0.0,-1.84,0.97,1.1]
  gpos= 0.01
  effort= [10,20,10,10,5]
  ct.Run('mikata.effort',effort)
  ct.robot.MoveGripper(gpos, blocking=blocking)
  ct.robot.MoveToQ(q_trg, dt, blocking=blocking)
  ct.robot.mikata.DisableTorque()

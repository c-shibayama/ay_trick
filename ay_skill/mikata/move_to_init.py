#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to an initial pose.
  Usage: mikata.move_to_init [DT [, BLOCKING]]'''
def Run(ct,*args):
  dt= args[0] if len(args)>0 else 4.0
  blocking= args[1] if len(args)>1 else True
  q_trg= [0.0,-0.26,-0.01,1.8]
  gpos= 0.04
  ct.robot.MoveToQ(q_trg, dt, blocking=blocking)
  ct.robot.MoveGripper(gpos, blocking=blocking)

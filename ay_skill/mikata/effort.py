#!/usr/bin/python
from core_tool import *
def Help():
  return '''Set effort of Mikata Arm.
  Usage: mikata.effort EFFORT
    EFFORT: Effort from 0 to 100. '''
def Run(ct,*args):
  effort= args[0]
  ct.robot.mikata.MoveTo({jname:q for (jname,q) in zip(ct.robot.JointNames(),ct.robot.Q(arm=0))}, blocking=False)
  if isinstance(effort,(int,float)):
    ct.robot.mikata.SetPWM({jname:effort for jname in ct.robot.JointNames()})
    ct.robot.EndEff().Move(ct.robot.EndEff().Position(), max_effort=effort)
  else:
    ct.robot.mikata.SetPWM({jname:e for jname,e in zip(ct.robot.JointNames(),effort)})
    ct.robot.EndEff().Move(ct.robot.EndEff().Position(), max_effort=effort[-1])

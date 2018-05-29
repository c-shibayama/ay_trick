#!/usr/bin/python
from core_tool import *
def Help():
  return '''Just call get_x1 which is an example of forward kinematics.
  Usage: ex.fk1 [ARM [, ANGLES]]
  '''
def Run(ct,*args):
  ct.Run('ex.get_x1',*args)

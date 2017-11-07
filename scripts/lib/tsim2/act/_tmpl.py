#!/usr/bin/python
from core_tool import *
def Help():
  return '''Primitive action (template) for ODE grasping and pouring simulation.
  Usage: Expected to be used from an execution context.'''

def Run(ct,*args):
  params= args[0]  #Parameters (dictionary) of this action

  return (SUCCESS_CODE, FAILURE_PRECOND, FAILURE_OTHER)[0]


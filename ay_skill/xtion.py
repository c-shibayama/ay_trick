#!/usr/bin/python
from core_tool import *
def Help():
  return '''Xtion control.
  Usage:
    xtion
    xtion 'clear'
      Remove connection.
    xtion 'setup'
      Setup connection.
    xtion 'slow'
      Slowdown the capturing speed.
    xtion 'fast'
      Speedup capturing (fastest).
  '''

def Setup(ct):
  return ct.AddDynConfig('xtion', '/camera/driver', time_out=3.0)

def Run(ct,*args):
  if len(args)>0:
    command= args[0]
    args= args[1:]
  else:
    command= 'clear'
    args= []

  if command=='setup':
    if not Setup(ct):  return False
    return True

  elif command=='clear':
    ct.DelDynConfig('xtion')

  elif command=='slow':
    if not Setup(ct):  return False
    params= {'data_skip': 30}
    ct.dynconfig.xtion.update_configuration(params)
    return True

  elif command=='fast':
    if not Setup(ct):  return False
    params= {'data_skip': 0}
    ct.dynconfig.xtion.update_configuration(params)
    return True

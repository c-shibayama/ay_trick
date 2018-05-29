#!/usr/bin/python
from core_tool import *
def Help():
  return '''Control display level such as text message and rviz markers.
  Note: Interpretation of display level depends on each script.
  Usage:
    displavel
    displavel 'show'
      Return current display level.
    displavel 'shownum'
      Return current display level as a number.
    displavel 'quiet'
      Set display level to 'quiet'(0) (no message other than error and important warning).
    displavel 'normal'
      Set display level to 'normal'(1) (moderate level).
    displavel 'verbose'
      Set display level to 'verbose'(2) (verbose, redundant, noisy; for debug).
  '''
def Run(ct,*args):
  if len(args)==0:  cmd= 'show'
  else:  cmd= args[0]

  if not ct.HasAttr(TMP,'displavel'):
    ct.SetAttr(TMP,'displavel', 'normal')

  if cmd=='show':
    return ct.GetAttr(TMP,'displavel')
  elif cmd=='shownum':
    return {'quiet':0,'normal':1,'verbose':2}[ct.GetAttr(TMP,'displavel')]
  elif cmd in ('quiet','normal','verbose'):
    ct.SetAttr(TMP,'displavel', cmd)
  else:
    raise Exception('displavel: Invalid argument.')

#!/usr/bin/python
from core_tool import *
def Help():
  return '''Control gripper position.
  Usage:
    grip POSITION
      Move gripper to POSITION (floating-point value).
    grip 'open'
      Open gripper.
    grip 'close'
      Close gripper.
    grip
    grip 'pos'
      Get current position.
'''
def Run(ct,*args):
  if len(args)==0 or args[0]=='pos':  print ct.robot.GripperPos()
  elif args[0]=='open':  ct.robot.OpenGripper()
  elif args[0]=='close':  ct.robot.CloseGripper()
  else:
    grange= ct.robot.GripperRange()
    gpos= float(args[0])
    if gpos<grange[0] or gpos>grange[1]:
      print 'Target gripper position is out of range.'
      print '  target:',gpos
      print '  range:',grange
      print 'Do you want to modify?'
      res= AskGen('y','n','c')
      if res=='y':
        gpos= min(grange[1],max(grange[0],gpos))
        print 'Modified to:',gpos
      elif res=='c':
        print 'Canceled'
        return
    ct.robot.MoveGripper(gpos)


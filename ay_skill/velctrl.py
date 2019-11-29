#!/usr/bin/python
from core_tool import *
def Help():
  return '''Return a velocity control interface of a robot.
  Usage: Don't run this script from CUI.
    velctrl [ARM]
        ARM: RIGHT or LEFT. Default: ct.robot.Arm

    velctrl= ct.Run('velctrl',ARM)  #ARM can be omitted
    try:
      while ...:
        dq= [...]
        velctrl.Step(dq)
    finally:
      velctrl.Finish()
'''
def Run(ct,*args):
  if ct.robot is None:
    raise Exception('Robot is not ready.')

  arm= args[0] if len(args)>0 else ct.robot.Arm

  if ct.robot.Is('Baxter'):
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      ct.m.velctrl= ct.Load('bx.velctrl')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct)
  elif ct.robot.Is('Motoman'):
    #if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      #ct.m.velctrl= ct.Load('moto.velctrl')
    #velctrl= ct.m.velctrl.TVelCtrl(arm,ct)
    raise Exception('Velocity control of {robot} does not work.'.format(robot=ct.robot.Name))
  elif ct.robot.Is('Mikata'):  #NOTE: This is also used for CraneX7.
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      ct.m.velctrl= ct.Load('mikata.velctrl_p')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct)
  elif ct.robot.Is('UR'):
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      if not ct.robot.Is('sim'):  ct.m.velctrl= ct.Load('ur.velctrl')
      else:                       ct.m.velctrl= ct.Load('ur.velctrl_ros')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct)
  elif ct.robot.Is('Gen3'):
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      ct.m.velctrl= ct.Load('gen3.velctrl')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct)
  else:
    raise Exception('{robot} does not support velocity control.'.format(robot=ct.robot.Name))

  print 'TVelCtrl.NumReferences=',ct.m.velctrl.TVelCtrl.NumReferences(arm)

  return velctrl

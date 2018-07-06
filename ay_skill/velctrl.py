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
    velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)
  elif ct.robot.Is('Motoman'):
    #velctrl= ct.Load('moto.velctrl').TVelCtrl(ct)
    raise Exception('Velocity control of {robot} does not work.'.format(robot=ct.robot.Name))
  elif ct.robot.Is('Mikata'):
    velctrl= ct.Load('mikata.velctrl_p').TVelCtrl(ct)
  elif ct.robot.Is('UR'):
    velctrl= ct.Load('ur.velctrl').TVelCtrl(ct,arm=arm)
  else:
    raise Exception('{robot} does not support velocity control.'.format(robot=ct.robot.Name))

  return velctrl

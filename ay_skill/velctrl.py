#!/usr/bin/python
from core_tool import *
def Help():
  return '''Return a velocity control interface of a robot.
  Usage: Don't run this script from CUI.
    velctrl [ARM [, RATE]]
        ARM: RIGHT or LEFT. Default: ct.robot.Arm
        RATE: Rate of control in Hz (default: None).
          None to use the robot default value.

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
  rate= args[1] if len(args)>1 else None

  velctrl= None
  if ct.robot.Is('Baxter'):
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      ct.m.velctrl= ct.Load('bx.velctrl')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct,rate=rate)
  elif ct.robot.Is('Motoman'):
    #if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      #ct.m.velctrl= ct.Load('moto.velctrl')
    #velctrl= ct.m.velctrl.TVelCtrl(arm,ct)
    raise Exception('Velocity control of {robot} does not work.'.format(robot=ct.robot.Name))
  elif ct.robot.Is('Mikata'):  #NOTE: This is also used for CraneX7.
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      ct.m.velctrl= ct.Load('mikata.velctrl_p')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct,rate=rate)
  elif ct.robot.Is('UR'):
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      if not ct.robot.Is('sim'):
        #ct.m.velctrl= ct.Load('ur.velctrl')
        #velctrl= ct.m.velctrl.TVelCtrl(arm,ct)
        ct.m.velctrl= ct.Load('ur.velctrl_ros2')
        if rate is None: rate= 500 if ct.robot.Is('E') else 125
      else:
        ct.m.velctrl= ct.Load('ur.velctrl_ros')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct,rate=rate)
  elif ct.robot.Is('Gen3'):
    if 'velctrl' not in ct.m or ct.m.velctrl.TVelCtrl.NumReferences(arm)==0:
      ct.m.velctrl= ct.Load('gen3.velctrl')
    velctrl= ct.m.velctrl.TVelCtrl(arm,ct,rate=rate)
  else:
    raise Exception('{robot} does not support velocity control.'.format(robot=ct.robot.Name))

  print 'TVelCtrl.NumReferences=',ct.m.velctrl.TVelCtrl.NumReferences(arm)

  return velctrl

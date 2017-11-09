#!/usr/bin/python
from core_tool import *
def Help():
  return '''Set parameters for controllers with FingerVision.
  Usage: template'''
def Run(ct,*args):
  ct.SetAttr('fv_ctrl','min_gstep', [0.0005,0.0005])
  for arm in (RIGHT,LEFT):
    if ct.robot.EndEff(arm).Is('BaxterEPG'):  ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.002
    elif ct.robot.EndEff(arm).Is('DxlGripper'):  ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.004

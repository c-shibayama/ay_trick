#!/usr/bin/python
from core_tool import *
def Help():
  return '''Display current fingertip pose (x,y,z,quaternion).
  Usage: q'''
def Run(ct,*args):
  lx_f= ct.GetAttr('wrist_'+LRToStrs(ct.robot.Arm),'lx')  #Fingertip pose (local pose in the wrist frame).
  return list(ct.robot.FK(x_ext=lx_f))

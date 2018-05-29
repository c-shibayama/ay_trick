#!/usr/bin/python
from core_tool import *
def Help():
  return '''Display current joint angles
  Usage: q'''
def Run(ct,*args):
  return list(ct.robot.Q())

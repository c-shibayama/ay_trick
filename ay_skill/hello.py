#!/usr/bin/python
from core_tool import *
def Help():
  return '''Hello-world script.
  Usage: hello'''
def Run(ct,*args):
  print 'Hello World!'
  print 'This is a script.'
  print 'We have a CoreTool object:', ct
  return 'Bye'

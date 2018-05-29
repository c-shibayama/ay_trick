#!/usr/bin/python
from core_tool import *
def Help():
  return '''Default script executed at the beginning.
    The Main function defined below is executed as a thread,
    which is registered as "_default".
  Usage: (AUTOMATICALLY EXECUTED)'''

import tf

def Main(ct,th_info):
  ct.br= tf.TransformBroadcaster()
  ct.SetAttr('default_frame', 'base')

def Run(ct,*args):
  print 'Loading _default...'
  ct.thread_manager.Add(name='_default', target=lambda th_info,ct=ct: Main(ct,th_info))

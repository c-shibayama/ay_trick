#!/usr/bin/python
from core_tool import *

#Activate a state machine as a thread
#TThreadInfo object is assigned to sm.ThreadInfo
def RunSMAsThread(ct, sm, name):
  target= lambda th_info: (
      sm.__dict__.__setitem__('ThreadInfo',th_info),
      sm.Run() )
  ct.thread_manager.Add(name=name, target=target)

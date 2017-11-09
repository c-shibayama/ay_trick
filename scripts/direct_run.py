#!/usr/bin/python
#\file    direct_run.py
#\brief   Directly run a specified motion script without running cui_tool.py.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.16, 2017
import sys
import roslib; roslib.load_manifest('ay_trick')
import rospy
import core_tool

if __name__ == '__main__':
  motion= sys.argv[1]
  #motion= 'tsim2.test_replay'

  rospy.init_node('direct_run')
  ct= core_tool.TCoreTool()

  print 'Running _default...'
  ct.Run('_default')
  print 'Waiting thread _default...'
  ct.thread_manager.Join('_default')

  print '+++Start running:',motion
  res= ct.Run(motion)
  if res!=None:  print 'Result:',res
  print '+++Finished running:',motion

  print 'Running _exit...'
  ct.Run('_exit')

  print 'TCoreTool.Cleanup...'
  ct.Cleanup()

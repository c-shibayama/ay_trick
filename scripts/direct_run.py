#!/usr/bin/python
#\file    direct_run.py
#\brief   Directly run motion scripts without running cui_tool.py.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.16, 2017
'''
Usage:
  rosrun ay_trick direct_run.py SCRIPTS
  scripts/direct_run.py SCRIPTS
    SCRIPTS: Scripts to be executed.
      For arguments, use quotation.
      Multiple lines can be combined by space.
    e.g.
      scripts/direct_run.py test.test
      scripts/direct_run.py 'test.test 3.14'
      scripts/direct_run.py 'test.test 3.14' 'test.test'
      scripts/direct_run.py 'robot "dxlg"' j
'''
import sys
import roslib; roslib.load_manifest('ay_trick')
import rospy
from core_tool import TCoreTool, CPrint
from cui_tool import ParseAndRun

if __name__ == '__main__':
  #motions= (' '.join(sys.argv[1:])).split(';')
  motions= sys.argv[1:]
  print motions
  #motions= ['tsim2.test_replay']

  rospy.init_node('direct_run')
  ct= TCoreTool()

  print 'Running _default...'
  ct.Run('_default')
  print 'Waiting thread _default...'
  ct.thread_manager.Join('_default')

  for motion in motions:
    CPrint(2,'+++Start running:',motion)
    #res= ct.Run(motion)
    res= ParseAndRun(ct, motion)
    if res!=None:  print 'Result:',res
    CPrint(2,'+++Finished running:',motion)

  print 'Running _exit...'
  ct.Run('_exit')

  print 'TCoreTool.Cleanup...'
  ct.Cleanup()

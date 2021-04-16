#! /usr/bin/env python
#\file    ros_node.py
#\brief   ROS node version of running motion script interface.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.16, 2021
import sys
import roslib;
roslib.load_manifest('ay_trick')
roslib.load_manifest('ay_trick_msgs')
roslib.load_manifest('ay_py')
import rospy
import std_msgs.msg
import std_srvs.srv
#import ay_trick_msgs.msg
import ay_trick_msgs.srv
from core_tool import TCoreTool
from cui_tool import SetCT,ParseAndRun
from ay_py.core import CPrint, PrintException, ToStdType, yamldump, YDumper

ct= None

class TROSNode(object):
  def __init__(self):
    global ct
    ct= TCoreTool()
    SetCT(ct)
    self._running= False
    self._result= None

    try:
      if ct.Exists('_default'):
        print 'Running _default...'
        self._running= True
        ct.Run('_default')
        print 'Waiting thread _default...'
        ct.thread_manager.Join('_default')
      else:
        print '(info: script _default does not exist)'
    except Exception as e:
      PrintException(e,' in ROSNode')
    finally:
      self._running= False

    self.srv_wait= rospy.Service('~wait_finish', std_srvs.srv.Empty, self.WaitFinish)
    self.srv_wait= rospy.Service('~get_result_as_yaml', ay_trick_msgs.srv.GetString, self.GetResultAsYAML)
    self.sub_cmd= rospy.Subscriber('~command', std_msgs.msg.String, self.CommandCallback)

  def __del__(self):
    global ct
    try:
      if ct.Exists('_exit'):
        print 'Running _exit...'
        self._running= True
        ct.Run('_exit')
      else:
        print '(info: script _exit does not exist)'
    except Exception as e:
      PrintException(e,' in ROSNode')
    finally:
      self._running= False
      print 'TCoreTool.Cleanup...'
      ct.Cleanup()
      ct= None

  def RunCommand(self, cmd):
    global ct
    try:
      CPrint(2,'+++Start running:',cmd)
      self._running= True
      res= ParseAndRun(ct, cmd)
      self._result= res
      if res!=None:  print 'Result:',res
      CPrint(2,'+++Finished running:',cmd)
    except Exception as e:
      PrintException(e,' in ROSNode')
    finally:
      self._running= False

  def CommandCallback(self, msg):
    self.RunCommand(msg.data)

  def WaitFinish(self, req):
    global ct
    rate= rospy.Rate(50)
    while self._running:
      rate.sleep()
    return std_srvs.srv.EmptyResponse()

  def GetResultAsYAML(self, req):
    global ct
    res= ay_trick_msgs.srv.GetStringResponse()
    data= self._result
    res.result= yamldump(ToStdType(data), Dumper=YDumper)
    return res

if __name__=='__main__':
  rospy.init_node('ros_node')
  ros_node= TROSNode()
  rospy.spin()

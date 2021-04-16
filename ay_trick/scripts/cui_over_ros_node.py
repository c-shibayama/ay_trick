#!/usr/bin/python
#\file    cui_over_ros_node.py
#\brief   CUI using the ROS node version of running motion script interface (TROSNode).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.17, 2021
import os
import roslib;
roslib.load_manifest('ay_trick')
roslib.load_manifest('ay_trick_msgs')
roslib.load_manifest('ay_py')
import rospy
import std_msgs.msg
import std_srvs.srv
import ay_trick_msgs.srv
from ay_py.core import CPrint, PrintException, ACol, ToStdType, yamldump, YDumper, yamlload, YLoader
import readline
import threading

class TCUIOverROSNode(object):

  def __init__(self):
    self.hist_file= os.environ['HOME']+'/.ay_trick_hist' #'.trick_hist'
    try:
      readline.read_history_file(self.hist_file)
    except IOError:
      pass
    readline.parse_and_bind('tab: complete')
    self.write_history_file= readline.write_history_file

    self.thread_cui= None
    self.running= False
    self.done_exit_proc= False

    self.srvp_wait_finish= rospy.ServiceProxy('/ros_node/wait_finish', std_srvs.srv.Empty, persistent=False)
    self.srvp_get_result_as_yaml= rospy.ServiceProxy('/ros_node/get_result_as_yaml', ay_trick_msgs.srv.GetString, persistent=False)
    self.srvp_get_attr_as_yaml= rospy.ServiceProxy('/ros_node/get_attr_as_yaml', ay_trick_msgs.srv.GetAttrAsString, persistent=False)
    self.srvp_set_attr_with_yaml= rospy.ServiceProxy('/ros_node/set_attr_with_yaml', ay_trick_msgs.srv.SetAttrWithString, persistent=False)
    self.pub_cmd= rospy.Publisher('/ros_node/command', std_msgs.msg.String, queue_size=10)

  def Exit(self,where='',wait_cui=True):
    self.running= False
    if wait_cui and self.thread_cui is not None:  self.thread_cui.join()

    #exit_locker= threading.RLock()
    #with exit_locker:
    if not self.done_exit_proc:
      self.SaveHistory()
      rospy.signal_shutdown('quit...')
      self.done_exit_proc= True

  def SaveHistory(self):
    #print 'Save history into ',self.hist_file
    #self.write_history_file(self.hist_file)
    pass

  def Start(self):
    self.thread_cui= threading.Thread(name='thread_cui', target=self.Interface)
    self.thread_cui.start()

  def Interface(self):
    self.running= True
    while self.running and not rospy.is_shutdown():
      caret= 'trick or quit> '
      try:
        cmd_raw= raw_input(caret).strip()
      except EOFError:
        self.running= False
        continue

      try:
        if cmd_raw=='':
          continue
        elif cmd_raw == 'quit' or cmd_raw == 'exit':
          self.running= False
          continue
        else:
          self.pub_cmd.publish(std_msgs.msg.String(cmd_raw))
          self.srvp_wait_finish()
          res= self.srvp_get_result_as_yaml()
          if res.success:
            res_value= yamlload(res.result, Loader=YLoader)
            if res_value is not None:  print res_value
          else:
            CPrint(4,res.message)
      except Exception as e:
        PrintException(e,' in CUI')
        c1,c2,ce= ACol.I(4,1,None)
        print '%sCheck the command line: %s%s %s' % (c1, c2,cmd_raw, ce)

    self.Exit('the end of TCUITool.Interface',wait_cui=False)


if __name__ == '__main__':
  rospy.init_node('cui_over_ros_node')
  cui= TCUIOverROSNode()
  cui.Start()
  rospy.spin()
  cui.Exit('__main__')


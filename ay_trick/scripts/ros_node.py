#! /usr/bin/env python
#\file    ros_node.py
#\brief   ROS node version of running motion script interface.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.16, 2021
import sys
import time
import roslib;
roslib.load_manifest('ay_trick')
roslib.load_manifest('ay_trick_msgs')
roslib.load_manifest('ay_py')
import rospy
import std_msgs.msg
import std_srvs.srv
import ay_trick_msgs.msg
import ay_trick_msgs.srv
from core_tool import TCoreTool
from cui_tool import SetCT,ParseAndRun
from ay_py.core import CPrint, PrintException, ToStdType, yamldump, YDumper, yamlload, YLoader

ct= None

#Capture stdout and send it as a ROS topic.
class TStdOutToTopic(object):
  def __init__(self, pub):
    self.pub= pub
  def __enter__(self):
    self._stdout= sys.stdout
    sys.stdout= self
  def __exit__(self, *args):
    sys.stdout= self._stdout
  def close(self):
    pass
  def write(self, s):
    self._stdout.write(s)
    self.pub.publish(std_msgs.msg.String(s))
  def flush(self):
    pass

#Write content of buf to stdin.
class TStdInFromBuffer(object):
  def __init__(self, buf):
    self.buf= buf
    self.is_curses_term= False
  def __enter__(self):
    self._stdin= sys.stdin
    sys.stdin= self
  def __exit__(self, *args):
    sys.stdin= self._stdin
  def close(self):
    pass
  def wait_readable(self, timeout=0):
    t_start= time.time()
    while len(self.buf)==0 and time.time()-t_start<timeout:
      time.sleep(0.05)
    return len(self.buf)>0
  def read(self, n=None):
    while len(self.buf)==0:
      time.sleep(0.05)
    echo= True
    if n is None:
      s= self.buf.pop(0)
    elif n==len(self.buf[0]):
      s= self.buf.pop(0)
    elif n<len(self.buf[0]):
      s= self.buf[0][:n]
      self.buf[0]= self.buf[0][n:]
    else:  #n>len(self.buf[0])
      s= self.buf.pop(0)
      if not self.is_curses_term:  sys.stdout.write(s)
      echo= False
      s+= self.read(n-len(s))
    if echo and not self.is_curses_term: sys.stdout.write(s)
    return s
  def readline(self, n=None):
    while len(self.buf)==0:
      time.sleep(0.05)
    echo= True
    if n is None:
      if '\n' in self.buf[0]:
        i= self.buf[0].index('\n')
        if i==len(self.buf[0])-1:
          s= self.buf.pop(0)
        else:
          s= self.buf[0][:i+1]
          self.buf[0]= self.buf[0][i+1:]
      else:
        s= self.buf.pop(0)
        if not self.is_curses_term:  sys.stdout.write(s)
        echo= False
        s+= self.readline()
    elif n==len(self.buf[0]):
      s= self.buf.pop(0)
    elif n<len(self.buf[0]):
      s= self.buf[0][:n]
      self.buf[0]= self.buf[0][n:]
    else:  #n>len(self.buf[0])
      s= self.buf.pop(0)
      if not self.is_curses_term:  sys.stdout.write(s)
      echo= False
      s+= self.readline(n-len(s))
    if echo and not self.is_curses_term: sys.stdout.write(s)
    return s
  def flush(self):
    pass

class TROSNode(object):
  def __init__(self):
    global ct
    ct= TCoreTool()
    SetCT(ct)
    self._running= False
    self._result= None
    self._success= False
    self._message= ''
    self._mode= ay_trick_msgs.msg.ROSNodeMode(ay_trick_msgs.msg.ROSNodeMode.BUSY)

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

    self.key_buffer= []
    self.pub_node_status= rospy.Publisher('~node_status', ay_trick_msgs.msg.ROSNodeMode, queue_size=10)
    self.pub_stdout= rospy.Publisher('~stdout', std_msgs.msg.String, queue_size=100)
    self.srv_wait_finish= rospy.Service('~wait_finish', std_srvs.srv.Empty, self.WaitFinish)
    self.srv_get_result_as_yaml= rospy.Service('~get_result_as_yaml', ay_trick_msgs.srv.GetString, self.GetResultAsYAML)
    self.srv_get_attr_as_yaml= rospy.Service('~get_attr_as_yaml', ay_trick_msgs.srv.GetAttrAsString, self.GetAttrAsYAML)
    self.srv_set_attr_with_yaml= rospy.Service('~set_attr_with_yaml', ay_trick_msgs.srv.SetAttrWithString, self.SetAttrWithYAML)
    self.sub_key= rospy.Subscriber('~stdin', std_msgs.msg.String, self.StdInCallback)
    self.sub_cmd= rospy.Subscriber('~command', std_msgs.msg.String, self.CommandCallback)

    self._mode.mode= ay_trick_msgs.msg.ROSNodeMode.READY

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
    self.WaitFinish()
    try:
      self._mode.mode= ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING
      self._result= None
      self._success= False
      self._message= ''
      CPrint(2,'+++Start running:',cmd)
      self._running= True
      self.key_buffer= []  #Clear key buffer.
      with TStdOutToTopic(self.pub_stdout), TStdInFromBuffer(self.key_buffer):
        res= ParseAndRun(ct, cmd)
      self._result= res
      self._success= True
      self._message= ''
      if res!=None:  print 'Result:',res
      CPrint(2,'+++Finished running:',cmd)
    except Exception as e:
      self._success= False
      self._message= repr(e)
      PrintException(e,' in ROSNode')
    finally:
      self._running= False
      self._mode.mode= ay_trick_msgs.msg.ROSNodeMode.READY

  def CommandCallback(self, msg):
    self.RunCommand(msg.data)

  def StdInCallback(self, msg):
    self.key_buffer.append(msg.data)

  def WaitFinish(self, req=None):
    global ct
    rate= rospy.Rate(50)
    while not rospy.is_shutdown() and self._running:
      rate.sleep()
    return std_srvs.srv.EmptyResponse()

  def GetResultAsYAML(self, req):
    global ct
    res= ay_trick_msgs.srv.GetStringResponse()
    data= self._result
    res.result= yamldump(ToStdType(data), Dumper=YDumper)
    res.success= self._success
    res.message= self._message
    return res

  def GetAttrAsYAML(self, req):
    global ct
    res= ay_trick_msgs.srv.GetAttrAsStringResponse()
    res.success= False
    res.message= ''
    data= None
    try:
      data= ct.GetAttr(*req.keys)
      res.success= True
    except Exception as e:
      res.message= repr(e)
      PrintException(e,' in ROSNode')
    res.result= yamldump(ToStdType(data), Dumper=YDumper)
    return res

  def SetAttrWithYAML(self, req):
    global ct
    res= ay_trick_msgs.srv.SetAttrWithStringResponse()
    res.success= False
    res.message= ''
    try:
      value= yamlload(req.value, Loader=YLoader)
      if isinstance(value,dict):
        ct.AddDictAttr(*(list(req.keys)+[value]))
      else:
        ct.SetAttr(*(list(req.keys)+[value]))
      res.success= True
    except Exception as e:
      res.message= repr(e)
      PrintException(e,' in ROSNode')
    return res

if __name__=='__main__':
  rospy.init_node('ros_node')
  rospy.sleep(0.1)
  ros_node= TROSNode()
  #rospy.spin()
  rate= rospy.Rate(20)
  while not rospy.is_shutdown():
    ros_node.pub_node_status.publish(ros_node._mode)
    rate.sleep()

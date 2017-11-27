#!/usr/bin/python
from core_tool import *
def Help():
  return '''Baxter range sensor test.
  Usage: bx.test.bxrange'''
def Callback(ct,msg):
  #print msg
  dist= float(msg.data)/1000.0

  #Pose of range sensor in robot frame:
  x_r= ct.robot.FK(x_ext=ct.GetAttr('left_hand_range','lx'), arm=LEFT)
  #Position of object (e.g. table) detected by the range sensor:
  x_o= Transform(x_r, [dist,0.0,0.0])
  print x_o

def Run(ct,*args):
  #Get a local pose of range sensor on wrist frame:
  ct.Run('tf_once', '/left_gripper', '/left_hand_range', ('left_hand_range','lx'))

  ct.AddSub('l_hand_range', '/robot/analog_io/left_hand_range/value_uint32', std_msgs.msg.UInt32,
           lambda msg,ct=ct: Callback(ct,msg) )
  rospy.sleep(1.0)
  ct.DelSub('l_hand_range')

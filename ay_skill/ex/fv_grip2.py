#!/usr/bin/python
from core_tool import *
import std_msgs
def Help():
  return '''Simple example of FingerVision-based robot control.
  Usage: ex.fv_grip2
    You need to run before executing this script:
    rosrun fv_ros_example color_detect_node _cam_config:=config/fv_3_l.yaml '''

def Callback(msg, ct):
  grange= ct.robot.GripperRange()
  red_area= msg.data
  g_trg= min(grange[1],max(grange[0],grange[1] - 5.0*red_area))
  print red_area, g_trg
  ct.robot.MoveGripper(g_trg,speed=100)

def Run(ct,*args):
  arm= 0

  #Subscribe the topic published by the FV plugin:
  ct.AddSub('color_ratio', '/color_detect_node/color_ratio', std_msgs.msg.Float64, Callback, ct)
  #sub= rospy.Subscriber('/color_detect_node/color_ratio', std_msgs.msg.Float64, Callback, ct)

  rate= rospy.Rate(20)  #HZ
  kbhit= TKBHit()
  try:
    while True:
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':
          break;
      else:
        break
      rate.sleep()

  finally:
    kbhit.Deactivate()
    ct.DelSub('color_ratio')
    #sub.unregister()


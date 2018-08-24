#!/usr/bin/python
from core_tool import *
def Help():
  return '''Simple example of FingerVision-based robot control.
  Usage: ex.fv_grip1'''
def Run(ct,*args):
  arm= 0
  grange= ct.robot.GripperRange()

  #If fv is not active, turn on it.
  if not all(ct.Run('fv.fv','is_active',arm)):
    ct.Run('fv.fv','on',arm)
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))

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

      obj_area= 0.5*(fv_data.obj_area[0] + fv_data.obj_area[1])
      g_trg= min(grange[1],max(grange[0],grange[1] - 5.0*obj_area))
      print obj_area, g_trg
      ct.robot.MoveGripper(g_trg,speed=100)
      rate.sleep()

  finally:
    kbhit.Deactivate()


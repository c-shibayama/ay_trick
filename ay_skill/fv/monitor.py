#!/usr/bin/python
from core_tool import *
def Help():
  return '''Monitor the FV data.
  Usage: fv.monitor VALUE1 [, VALUE2 [, VALUE3 [, ...]]]
    VALUE?: Element of FV data.  We can use an equation.
    Press q or space to stop.
  Example:
    fv.monitor 'force[0]'
    fv.monitor 'np.mean(force,0)'
    fv.monitor 'sum(mv_s[0])+sum(mv_s[1])'    #slip
    fv.monitor 'sum(mv_s[0])/(obj_area_filtered[0]+1e-5)+sum(mv_s[1])/(obj_area_filtered[1]+1e-5)'    #slip normalized by the object area
    fv.monitor 'd_obj_center_filtered'
    fv.monitor 'map(lambda x:map(lambda y:round(y,2),x),d_obj_center_filtered)'
    fv.monitor 'map(lambda x:round(x,2),d_obj_orientation_filtered)'
    fv.monitor 'map(lambda x:round(x,2),d_obj_area_filtered)'
    fv.monitor 'np.max(d_obj_center_filtered)>0.05'
    fv.monitor 'np.max(d_obj_orientation_filtered)>0.4'
    fv.monitor 'np.max(d_obj_area_filtered)>0.1'
'''
def Run(ct,*args):
  assert(len(args)>0)
  if isinstance(args[0],int):  arm,values= args[0],args[1:]
  elif StrToLR(args[0]) is not None:  arm,values= StrToLR(args[0]),args[1:]
  elif StrToID(args[0]) is not None:  arm,values= StrToID(args[0]),args[1:]
  else:  arm,values= ct.robot.Arm,args

  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))
  def Monitor(ct, l, side):
    res= []
    for v in values:
      res.append(eval(v,globals(),fv_data.__dict__))
    line= '; '.join(map(repr,res))
    print line

  ct.callback.fv_objinfo[ct.robot.ArmStrS(arm)]= [Monitor,Monitor]

  try:
    #Stop object detection
    ct.Run('fv.fv','stop_detect_obj',arm)
    kbhit= TKBHit()
    while not rospy.is_shutdown():
      if kbhit.KBHit() in ('q',' '):  break
      rospy.sleep(0.01)
  finally:
    kbhit.Deactivate()
    ct.callback.fv_objinfo[ct.robot.ArmStrS(arm)]= [None,None]
    #Resume object detection
    ct.Run('fv.fv','start_detect_obj',arm)

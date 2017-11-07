#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of cutting motion.
  Usage: fv.cut1 [ARM]'''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else ct.robot.Arm
  dz= 0.01
  dt= 0.01
  lw_xe= ct.GetAttr('wrist_'+LRToStrs(arm),'lx')
  xe0= ct.robot.FK(x_ext=lw_xe,arm=arm)
  '''
  #Find an axis whose direction is close to z
  exyz= RotToExyz(QToRot(xe0[3:]))
  axis= None
  for e in exyz:
    if abs(e[2])>0.8:  axis= e
  if axis is None:
    CPrint(4,'Can not find a moving direction')
    return
  #Modify xe0 so that its orientation becomes upright
  qmod= QFromAxisAngle(*GetAxisAngle(axis,[0.0,0.0,1.0 if axis[2]>0.0 else -1.0]))
  xe0[3:]= MultiplyQ(qmod,xe0[3:])
  '''
  #Target position
  xe1= copy.deepcopy(xe0)
  xe1[2]-= dz
  #Move
  ct.robot.MoveToX(xe1,dt,lw_xe,blocking=True,arm=arm)

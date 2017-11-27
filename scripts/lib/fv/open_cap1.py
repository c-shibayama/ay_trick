#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of opening a cap.
  Usage:
    fv.open_cap1 '''

def OpenCapLoop(ct, arm):
  #vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  wrist= ['wrist_r','wrist_l'][arm]
  x_ext= ct.GetAttr(wrist,'lx')
  x_e0= ct.robot.FK(x_ext=x_ext,arm=arm)
  ex0,ey0,ez0= RotToExyz(QToRot(x_e0[3:]))
  omega= -0.2  #Rotational velocity

  velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)

  try:
    for tm in np.arange(0.0,5.0,1.0/velctrl.rate):
      if rospy.is_shutdown():
        break

      #x_e= ct.robot.FK(x_ext=x_ext,arm=arm)
      #ex,ey,ez= RotToExyz(QToRot(x_e[3:]))
      #vel= [vel_x(),vel_y(),vel_z()]

      q= ct.robot.Q(arm=arm)
      J= ct.robot.J(q,arm=arm)
      #vq0= ct.robot.limbs[arm].joint_velocities()
      #vq0= MCVec([vq0[joint] for joint in ct.robot.JointNames(arm)])
      #vx0= J * vq0
      vx= MCVec([0.0]*3 + ToList(omega*ex0))

      dq= ToList(la.pinv(J)*vx)
      #dq= [0.0]*6+[0.7]
      #dq= [0.0, 0.0, 0.0, 0.0, 0.37, 0.02, 0.80]
      print ' '.join(map(lambda f:'%0.2f'%f,dq))

      velctrl.Step(dq)

  finally:
    velctrl.Finish()
    #ct.Run('fv.finger3','start_detect_obj',arm)

def Run(ct,*args):
  OpenCapLoop(ct, LEFT)

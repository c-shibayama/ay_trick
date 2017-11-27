#!/usr/bin/python
from core_tool import *
def Help():
  return '''Track an external force.
  Usage:
    fv.trackf 'on' [, ARM]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.trackf 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.trackf
    fv.trackf 'clear'
      Stop all threads. Equivalent to:
        fv.trackf 'off' LEFT
        fv.trackf 'off' RIGHT'''
def TrackingLoop(th_info, ct, arm):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  ct.Run('fv.finger3','stop_detect_obj',arm)

  f0= Vec(copy.deepcopy(vs_finger.force))
  f_diff= lambda side: Vec(vs_finger.force[side]) - f0[side]


  velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)

  try:
    wrist= ['wrist_r','wrist_l'][arm]
    while th_info.IsRunning() and not rospy.is_shutdown():
      x_ext= ct.GetAttr(wrist,'lx')
      x_e= ct.robot.FK(x_ext=x_ext,arm=arm)
      #ex,ey,ez= RotToExyz(QToRot(x_e[3:]))
      fd_l= 0.5*(f_diff(0)+f_diff(1))
      #fd_l= f_diff(1)
      fd= ToList(Transform(x_e[3:],fd_l[:3])) + [0,0,0]
      #fd= ToList(Transform(x_e[3:],fd_l[:3])) + ToList(Transform(x_e[3:],fd_l[3:]))
      #print Transform(x_e[3:],fd_l[:3]), f_diff(1)[:3]

      q= ct.robot.Q(arm=arm)
      J= ct.robot.J(q,arm=arm)
      vq0= ct.robot.limbs[arm].joint_velocities()
      vq0= MCVec([vq0[joint] for joint in ct.robot.JointNames(arm)])
      vx0= J * vq0
      #kv= np.diag([0.3,0.5,0.5])
      #vx= ToList(MCVec(vp) - kv*vx0[:3])+[0.0,0.0,0.0]
      vx= 0.2*MCVec(fd)
      dq= ToList(la.pinv(J)*vx)
      velctrl.Step(dq)

  finally:
    velctrl.Finish()
    ct.Run('fv.finger3','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    if 'vs_trackf'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_trackf'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    print 'Turn on:','vs_trackf'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_trackf'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_trackf'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_trackf'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_trackf'+LRToStrS(RIGHT)
    print 'Turn off:','vs_trackf'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_trackf'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_trackf'+LRToStrS(LEFT))


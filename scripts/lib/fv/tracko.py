#!/usr/bin/python
from core_tool import *
def Help():
  return '''Track an object based on object detection.
  Usage:
    fv.tracko 'on' [, ARM]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.tracko 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.tracko
    fv.tracko 'clear'
      Stop all threads. Equivalent to:
        fv.tracko 'off' LEFT
        fv.tracko 'off' RIGHT'''
def TrackingLoop(th_info, ct, arm):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  ct.Run('fv.finger3','stop_detect_obj',arm)

  center0= [0.5*(vs_finger.obj_center[0][0]-vs_finger.obj_center[1][0]),
            0.5*(vs_finger.obj_center[0][1]+vs_finger.obj_center[1][1]) ]
  obj_area0= 0.5*(vs_finger.obj_area[0] + vs_finger.obj_area[1])

  def out_of_track():
    obj_area= 0.5*(vs_finger.obj_area[0] + vs_finger.obj_area[1])
    #print obj_area0, obj_area, (obj_area < 0.02*obj_area0)
    if obj_area < 0.02*obj_area0:  return True
    return False

  def vel_z():
    #th= 0.01
    y= 0.5*(vs_finger.obj_center[0][1]+vs_finger.obj_center[1][1])
    #return -1.0 if y>th else (+1.0 if y<-th else 0.0)
    return -y # if abs(y)>th else 0.0

  def vel_x():
    #th= 0.003
    #obj_s= [np.array(vs_finger.obj_s[0]), np.array(vs_finger.obj_s[1])]
    #o1= sum(obj_s[0][[2,5,8]]) + sum(obj_s[1][[0,3,6]])
    ##o2= sum(obj_s[0][[0,3,6]]) + sum(obj_s[1][[2,5,8]])
    #o2= sum(obj_s[0][[1,4,7]]) + sum(obj_s[1][[1,4,7]])
    #return +1.0 if (o1-o2)>th else (-1.0 if (o2-o1)>th else 0.0)
    #th= 0.001
    dx= 0.5*(vs_finger.obj_center[0][0]-vs_finger.obj_center[1][0]) - center0[0]
    #return +1.0 if dx>th else (-1.0 if dx<-th else 0.0)
    return dx # if abs(dx)>th else 0.0

  def vel_y():
    #th= 0.001
    a1= vs_finger.obj_area[0]
    a2= vs_finger.obj_area[1]
    #return +1.0 if (a2-a1)>th else (-1.0 if (a1-a2)>th else 0.0)
    return (a2-a1) # if abs(a2-a1)>th else 0.0

  velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)

  try:
    wrist= ['wrist_r','wrist_l'][arm]
    while th_info.IsRunning() and not rospy.is_shutdown():
      x_ext= ct.GetAttr(wrist,'lx')
      x_e= ct.robot.FK(x_ext=x_ext,arm=arm)
      ex,ey,ez= RotToExyz(QToRot(x_e[3:]))
      #print vel_x(),vel_y(),vel_z()
      gain= [0.5,0.5,0.5]
      #gain= [1.0,1.6,1.0]
      vel= [vel_x(),vel_y(),vel_z()]
      #print vel
      vp= gain[0]*vel[0]*ex + gain[1]*vel[1]*ey + gain[2]*vel[2]*ez
      #print gain,vel
      #print '---',gain[0]*vel[0]*ex,gain[1]*vel[1]*ey,gain[2]*vel[2]*ez, vp
      #x_trg= copy.deepcopy(x_e)
      #x_trg[:3]+= vp
      #print x_trg
      #ct.robot.MoveToX(x_trg,0.1,x_ext,arm=arm,blocking='time')
      #rospy.sleep(0.1)
      #angles= ct.robot.IK(x_trg,x_ext,arm=arm)
      #if angles is not None:
        #ct.robot.FollowQTraj([angles],[0.01],arm=arm,blocking='time')
      #else:
        #CPrint(4,'IK error')

      if out_of_track():
        dq= [0.0]*ct.robot.DoF(arm)
      else:
        q= ct.robot.Q(arm=arm)
        J= ct.robot.J(q,arm=arm)
        vq0= ct.robot.limbs[arm].joint_velocities()
        vq0= MCVec([vq0[joint] for joint in ct.robot.JointNames(arm)])
        vx0= J * vq0
        kv= np.diag([0.3,0.3,0.5])
        vx= ToList(MCVec(vp) - kv*vx0[:3])+[0.0,0.0,0.0]
        dq= ToList(la.pinv(J)*MCVec(vx))
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
    if 'vs_tracko'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_tracko'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    print 'Turn on:','vs_tracko'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_tracko'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_tracko'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_tracko'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_tracko'+LRToStrS(RIGHT)
    print 'Turn off:','vs_tracko'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_tracko'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_tracko'+LRToStrS(LEFT))


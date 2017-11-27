#!/usr/bin/python
from core_tool import *
def Help():
  return '''Track an external force.  Using posforce_array and Baxter's endpoint force estimate.
    This is a best Tai Chi code with FingerVision and Baxter's force estimate.
  Usage:
    fv.trackf4 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    fv.trackf4 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.trackf4
    fv.trackf4 'clear'
      Stop all threads. Equivalent to:
        fv.trackf4 'off' LEFT
        fv.trackf4 'off' RIGHT'''
#def VSFFilter(ct, vs_finger, side):
  #...
#ct.callback.vs_finger_bm[LRToStrS(arm)]= [VSFFilter,VSFFilter]

def WrenchToList2(wrench,l):
  force= wrench['force']
  torque= wrench['torque']
  l[0]= force.x
  l[1]= force.y
  l[2]= force.z
  l[3]= torque.x
  l[4]= torque.y
  l[5]= torque.z

def TrackingLoop(th_info, ct, arm, ctrl_type):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  #ct.Run('fv.finger3','stop_detect_obj',arm)

  #f0= Vec(copy.deepcopy(vs_finger.force))
  #f_diff= lambda side: Vec(vs_finger.force[side]) - f0[side]

  pfa_scale= [None,None]
  pfa0= [None,None]
  pfa_filtered= [None,None]
  t_finit= [None]
  def UpdateF0():
    for side in (RIGHT,LEFT):
      pfa_scale[side]= [max(1.0, la.norm(p_f[2:])/2.0) for p_f in vs_finger.posforce_array[side]]
      #pfa0= copy.deepcopy(vs_finger.posforce_array)
      pfa0[side]= [[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(vs_finger.posforce_array[side], pfa_scale[side])]
      pfa_filtered[side]= copy.deepcopy(pfa0[side])
      ##FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
      #n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(vs_finger.posforce_array[side],fa0[side])])
    t_finit[0]= rospy.Time.now()
  UpdateF0()

  def StepFilter(side):
    #We will consider only [fx,fz] since they are most reliable.
    th_f= 5.0
    #Each element in posforce_array is [x,z,fx,fy,fz]
    #pfa= vs_finger.posforce_array[side]
    pfa= [[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(vs_finger.posforce_array[side], pfa_scale[side])]
    num_tracking= len(pfa_filtered[side])
    pfa_filtered[side]= [p_f if (abs(p_f[2]-p_f_f[2])<th_f and abs(p_f[4]-p_f_f[4])<th_f)
                              else None
                         for p_f,p_f0,p_f_f in zip(pfa,pfa0[side],pfa_filtered[side]) if p_f_f is not None]
    if num_tracking>len(pfa_filtered[side]):
      CPrint(0,'Lost some points in tracking,',vs_finger.vs_finger,LRToStrS(side),len(pfa_filtered[side]))
  warned= [False,False]
  def FDiff():
    force_array= []
    for side in (RIGHT,LEFT):
      diff_pfa= [[p_f_f[0],p_f_f[1], p_f_f[2]-p_f0[2],p_f_f[3]-p_f0[3],p_f_f[4]-p_f0[4]]
                for p_f0,p_f_f in zip(pfa0[side],pfa_filtered[side]) if p_f_f is not None]
      if len(diff_pfa)==0:
        if not warned[side]:
          CPrint(4,'All points are out of track,',vs_finger.vs_finger,LRToStrS(side))
          warned[side]= True
        return Vec([0.0]*6)
      gpos= (-1.0,1.0)[side]*ct.robot.GripperPos(arm)
      force_array+= [p_f[2:] + np.cross([p_f[0],gpos,p_f[1]],p_f[2:]).tolist() for p_f in diff_pfa]
    f_diff= [sum([force[d] for force in force_array])/float(len(force_array)) for d in xrange(6)]
    return Vec(f_diff)

  def out_of_track():
    #return False
    obj_area= 0.5*(vs_finger.obj_area[0] + vs_finger.obj_area[1])
    #print obj_area
    if obj_area < 0.02:  return True
    return False

  wrench= [0.0]*6
  wrench0= None

  velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)
  time0= rospy.Time.now()

  try:
    wrist= ['wrist_r','wrist_l'][arm]
    while th_info.IsRunning() and not rospy.is_shutdown():
      if (rospy.Time.now()-time0).to_sec()>2.0:
        WrenchToList2(ct.robot.limbs[arm].endpoint_effort(), wrench)
        if wrench0 is None:
          wrench0= Vec(copy.deepcopy(wrench))
        if out_of_track():
          UpdateF0()
          wrench0= Vec(copy.deepcopy(wrench))

        x_ext= ct.GetAttr(wrist,'lx')
        x_e= ct.robot.FK(x_ext=x_ext,arm=arm)
        StepFilter(0)
        StepFilter(1)
        f_diff= FDiff()
        f_gain= 1.0
        t_gain= 5.0
        fd_l= ToList(Transform(x_e[3:],f_gain*f_diff[:3])) + ToList(Transform(x_e[3:],t_gain*f_diff[3:]))

        f_diff2= -(Vec(wrench) - wrench0)
        f_gain2= 0.05
        t_gain2= 0.3
        fd_l[:3]= Vec(fd_l[:3]) + f_gain2*f_diff2[:3]
        fd_l[3:]= Vec(fd_l[3:]) + t_gain2*f_diff2[3:]

        if ctrl_type=='position':
          fd= fd_l[:3] + [0.0,0.0,0.0]
        elif ctrl_type=='pose':
          fd= fd_l
        elif ctrl_type=='orientation':
          fd= [0.0,0.0,0.0] + fd_l[3:]
      else:
        fd= [0.0]*6

      if out_of_track():
        dq= [0.0]*ct.robot.DoF(arm)
      else:
        q= ct.robot.Q(arm=arm)
        J= ct.robot.J(q,arm=arm)
        #vq0= ct.robot.limbs[arm].joint_velocities()
        #vq0= MCVec([vq0[joint] for joint in ct.robot.JointNames(arm)])
        #vx0= J * vq0
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
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'vs_trackf4'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_trackf4'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    print 'Turn on:','vs_trackf4'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_trackf4'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm,ctrl_type))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_trackf4'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_trackf4'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_trackf4'+LRToStrS(RIGHT)
    print 'Turn off:','vs_trackf4'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_trackf4'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_trackf4'+LRToStrS(LEFT))


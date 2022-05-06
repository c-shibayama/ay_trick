#!/usr/bin/python
from core_tool import *
def Help():
  return '''Track an external force.  Using posforce_array.
    This is a best Tai Chi code with FingerVision.
  Usage:
    fv.trackf2 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    fv.trackf2 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.trackf2
    fv.trackf2 'clear'
      Stop all threads. Equivalent to:
        fv.trackf2 'off' LEFT
        fv.trackf2 'off' RIGHT'''

def TrackingLoop(th_info, ct, arm, ctrl_type):
  ct.Run('fv.ctrl_params')
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))

  #Stop object detection
  #ct.Run('fv.fv','stop_detect_obj',arm)

  #f0= Vec(copy.deepcopy(fv_data.force))
  #f_diff= lambda side: Vec(fv_data.force[side]) - f0[side]

  pfa_scale= [None,None]
  pfa0= [None,None]
  pfa_filtered= [None,None]
  t_finit= [None]
  def UpdateF0():
    for side in (RIGHT,LEFT):
      pfa_scale[side]= [max(1.0, la.norm(p_f[2:])/2.0) for p_f in fv_data.posforce_array[side]]
      #pfa0= copy.deepcopy(fv_data.posforce_array)
      pfa0[side]= [[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(fv_data.posforce_array[side], pfa_scale[side])]
      pfa_filtered[side]= copy.deepcopy(pfa0[side])
      ##FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
      #n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(fv_data.posforce_array[side],fa0[side])])
    t_finit[0]= rospy.Time.now()
  UpdateF0()

  def StepFilter(side):
    #We will consider only [fx,fz] since they are most reliable.
    th_f= 5.0
    #Each element in posforce_array is [x,z,fx,fy,fz]
    #pfa= fv_data.posforce_array[side]
    pfa= [[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(fv_data.posforce_array[side], pfa_scale[side])]
    num_tracking= len(pfa_filtered[side])
    pfa_filtered[side]= [p_f if (abs(p_f[2]-p_f_f[2])<th_f and abs(p_f[4]-p_f_f[4])<th_f)
                              else None
                         for p_f,p_f0,p_f_f in zip(pfa,pfa0[side],pfa_filtered[side]) if p_f_f is not None]
    if num_tracking>len(pfa_filtered[side]):
      CPrint(0,'Lost some points in tracking,',fv_data.fv,LRToStrS(side),len(pfa_filtered[side]))
  warned= [False,False]
  def FDiff():
    force_array= []
    for side in (RIGHT,LEFT):
      diff_pfa= [[p_f_f[0],p_f_f[1], p_f_f[2]-p_f0[2],p_f_f[3]-p_f0[3],p_f_f[4]-p_f0[4]]
                for p_f0,p_f_f in zip(pfa0[side],pfa_filtered[side]) if p_f_f is not None]
      if len(diff_pfa)==0:
        if not warned[side]:
          CPrint(4,'All points are out of track,',fv_data.fv,LRToStrS(side))
          warned[side]= True
        return Vec([0.0]*6)
      gpos= (-1.0,1.0)[side]*ct.robot.GripperPos(arm)
      force_array+= [p_f[2:] + np.cross([p_f[0],gpos,p_f[1]],p_f[2:]).tolist() for p_f in diff_pfa]
    f_diff= [sum([force[d] for force in force_array])/float(len(force_array)) for d in xrange(6)]
    return Vec(f_diff)

  def out_of_track():
    #return False
    obj_area= max(fv_data.obj_area[0],fv_data.obj_area[1])
    print obj_area
    if obj_area < 0.01:  return True
    return False

  vx_list= []  #List of target task space velocities for temporal filtering.
  smoothing_filter_len= ct.GetAttr('fv_ctrl','trackf2_flen')

  try:
    velctrl= ct.Run('velctrl',arm)
    print velctrl
    wrist= ['wrist_r','wrist_l'][arm]
    while th_info.IsRunning() and not rospy.is_shutdown():
      if out_of_track():
        UpdateF0()

      x_ext= ct.GetAttr(wrist,'lx')
      x_e= ct.robot.FK(x_ext=x_ext,arm=arm)
      #ex,ey,ez= RotToExyz(QToRot(x_e[3:]))
      StepFilter(0)
      StepFilter(1)
      #fd_l= 0.5*(FDiff(0)+FDiff(1))
      fd_l= FDiff()
      #fd_l= f_diff(1)
      t_gain= 5.0
      if ctrl_type=='position':
        fd= ToList(Transform(x_e[3:],fd_l[:3])) + [0.0,0.0,0.0]
      elif ctrl_type=='pose':
        fd= ToList(Transform(x_e[3:],fd_l[:3])) + ToList(Transform(x_e[3:],t_gain*fd_l[3:]))
      elif ctrl_type=='orientation':
        fd= [0.0,0.0,0.0] + ToList(Transform(x_e[3:],t_gain*fd_l[3:]))
      #print Transform(x_e[3:],fd_l[:3]), f_diff(1)[:3]

      if out_of_track():
        dq= [0.0]*ct.robot.DoF(arm)
      else:
        q= ct.robot.Q(arm=arm)
        J= ct.robot.J(q,arm=arm)
        vq0= MCVec(ct.robot.DQ(arm=arm))
        vx0= J * vq0
        #kv= np.diag([0.3,0.5,0.5])
        #vx= ToList(MCVec(vp) - kv*vx0[:3])+[0.0,0.0,0.0]
        vx= MCVec(ct.GetAttr('fv_ctrl','trackf2_kp')*np.array(fd))
        if smoothing_filter_len>1:
          vx_list.append(vx)
          if len(vx_list)>smoothing_filter_len:  vx_list.pop(0)
          vx= np.mean(np.hstack(vx_list),axis=1)
        if ct.robot.DoF(arm=arm)>=6:
          dq= ToList(la.pinv(J)*vx)
        else:  #e.g. Mikata Arm
          W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
          dq= ToList(la.pinv(W*J)*W*vx)
      velctrl.Step(dq)

  finally:
    velctrl.Finish()
    ct.Run('fv.fv','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'vs_trackf2'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_trackf2'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    CPrint(1,'Turn on:','vs_trackf2'+LRToStrS(arm))
    ct.thread_manager.Add(name='vs_trackf2'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm,ctrl_type))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    CPrint(2,'Turn off:','vs_trackf2'+LRToStrS(arm))
    ct.thread_manager.Stop(name='vs_trackf2'+LRToStrS(arm))

  elif command=='clear':
    CPrint(2,'Turn off:','vs_trackf2'+LRToStrS(RIGHT))
    CPrint(2,'Turn off:','vs_trackf2'+LRToStrS(LEFT))
    ct.thread_manager.Stop(name='vs_trackf2'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_trackf2'+LRToStrS(LEFT))


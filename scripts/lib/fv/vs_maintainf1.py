#!/usr/bin/python
from core_tool import *
def Help():
  return '''Minimizing external force.  Using posforce_array, and CMA-ES.
  Usage:
    fv.vs_maintainf1 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    fv.vs_maintainf1 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vs_maintainf1
    fv.vs_maintainf1 'clear'
      Stop all threads. Equivalent to:
        fv.vs_maintainf1 'off' LEFT
        fv.vs_maintainf1 'off' RIGHT'''
#def VSFFilter(ct, vs_finger, side):
  #...
#ct.callback.vs_finger_bm[LRToStrS(arm)]= [VSFFilter,VSFFilter]

def TrackingLoop(th_info, ct, arm, ctrl_type):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  #ct.Run('fv.vs_finger3','stop_detect_obj',arm)

  #f0= Vec(copy.deepcopy(vs_finger.force))
  #f_diff= lambda side: Vec(vs_finger.force[side]) - f0[side]

  #pfa_scale= [[max(1.0, la.norm(p_f[2:])/2.0) for p_f in vs_finger.posforce_array[side]] for side in range(2)]
  pfa0= copy.deepcopy(vs_finger.posforce_array)
  #pfa0= [[[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(vs_finger.posforce_array[side], pfa_scale[side])] for side in range(2)]
  pfa_filtered= copy.deepcopy(pfa0)
  ##FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
  #n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(vs_finger.posforce_array[side],fa0[side])])

  def StepFilter():
    #We will consider only [fx,fz] since they are most reliable.
    th_f= 100.0
    for side in (RIGHT,LEFT):
      #Each element in posforce_array is [x,z,fx,fy,fz]
      pfa= vs_finger.posforce_array[side]
      #pfa= [[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(vs_finger.posforce_array[side], pfa_scale[side])]
      num_tracking= len(pfa_filtered[side])
      pfa_filtered[side]= [p_f if (abs(p_f[2]-p_f_f[2])<th_f and abs(p_f[4]-p_f_f[4])<th_f)
                                else None
                          for p_f,p_f0,p_f_f in zip(pfa,pfa0[side],pfa_filtered[side]) if p_f_f is not None]
      if num_tracking>len(pfa_filtered[side]):
        CPrint(0,'Lost some points in tracking,',vs_finger.vs_finger,LRToStrS(side),len(pfa_filtered[side]))
  warned= [False,False]
  def FNorm():
    f_norm= []
    for side in (RIGHT,LEFT):
      fa= [[p_f_f[2],p_f_f[3],p_f_f[4]] for p_f_f in pfa_filtered[side] if p_f_f is not None]
      if len(fa)==0:
        if not warned[side]:
          CPrint(4,'All points are out of track,',vs_finger.vs_finger,LRToStrS(side))
          warned[side]= True
        return Vec([0.0]*6)
      f_norm+= [NormSq(f) for f in fa]
    f_norm= sum(f_norm)/len(f_norm)
    return f_norm

  wrist= ['wrist_r','wrist_l'][arm]
  x_ext= ct.GetAttr(wrist,'lx')
  xe0= ct.robot.FK(x_ext=x_ext,arm=arm)

  velctrl= ct.Load('bx_velctrl').TVelCtrl(ct,arm=arm)

  opt= TContOptNoGrad()
  options= {}
  options['bounds']= [[-0.2]*3,[0.2]*3]
  options['tolfun']= 1.0e-4
  options['scale0']= 0.005
  #options['popsize']= 3
  options['parameters0']= [0.0]*3
  opt.Init({'options':options})
  count= 0

  try:
    while th_info.IsRunning() and not rospy.is_shutdown():
      StepFilter()
      N= 50
      if count==0:
        f_norm_sum= 0.0
        dp= opt.Select()
      elif count%N==0:
        f_norm_sum+= FNorm()
        f_norm_sum/= float(N)
        opt.Update(-f_norm_sum)
        print dp,f_norm_sum
        f_norm_sum= 0.0
        dp= opt.Select()
      else:
        f_norm_sum+= FNorm()
      count+= 1

      q= ct.robot.Q(arm=arm)
      xe1= ct.robot.FK(q,x_ext=x_ext,arm=arm)
      J= ct.robot.J(q,arm=arm)
      vq1= ct.robot.limbs[arm].joint_velocities()
      vq1= MCVec([vq1[joint] for joint in ct.robot.JointNames(arm)])
      vx1= J * vq1
      kp= np.diag([1.0,1.0,1.0])
      #kv= np.diag([0.3,0.5,0.5])
      kv= np.diag([0.0,0.0,0.0])
      vx= ToList(kp*(MCVec(xe0[:3])+MCVec(dp)-MCVec(xe1[:3])) - kv*vx1[:3])+[0.0,0.0,0.0]
      dq= ToList(la.pinv(J)*MCVec(vx))
      velctrl.Step(dq)

  finally:
    velctrl.Finish()
    ct.Run('fv.vs_finger3','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'vs_maintainf1'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_maintainf1'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.vs_finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.vs_finger3','setup',arm)

    ct.Run('fv.vs_grasp','off',arm)
    ct.Run('fv.vs_hold','off',arm)

    print 'Turn on:','vs_maintainf1'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_maintainf1'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm,ctrl_type))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_maintainf1'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_maintainf1'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_maintainf1'+LRToStrS(RIGHT)
    print 'Turn off:','vs_maintainf1'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_maintainf1'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_maintainf1'+LRToStrS(LEFT))

#!/usr/bin/python
from core_tool import *
def Help():
  return '''Minimizing external force.  Using posforce_array, linear regression, and QP.
  Usage:
    fv.maintainf2 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    fv.maintainf2 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.maintainf2
    fv.maintainf2 'clear'
      Stop all threads. Equivalent to:
        fv.maintainf2 'off' LEFT
        fv.maintainf2 'off' RIGHT'''
#def VSFFilter(ct, vs_finger, side):
  #...
#ct.callback.vs_finger_bm[LRToStrS(arm)]= [VSFFilter,VSFFilter]

def UpdateWeight(W, x, y, alpha=0.1):
  xi= MCVec(AddOne(x))
  y= MCVec(y)
  err= y - W.T.dot(xi)
  W+= alpha*xi.dot(err.T)

def UpdateWeightMB(W, x, y, x_data, y_data, alpha=0.1, n_batch=20, beta=0.001):
  x_data.append(x)
  y_data.append(y)
  if len(x_data)>n_batch:
    x_data.pop(0)
    y_data.pop(0)
  Xi= np.mat([AddOne(x) for x in x_data])
  Y= np.mat(y_data)
  Wnew= (Xi.T.dot(Xi)+beta*np.eye(len(x)+1)).I.dot(Xi.T.dot(Y))
  W*= (1.0-alpha)
  W+= alpha*Wnew

def TrackingLoop(th_info, ct, arm, ctrl_type):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  #ct.Run('fv.finger3','stop_detect_obj',arm)

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
  def Farray():
    fa= []
    for side in (RIGHT,LEFT):
      fa+= sum([[p_f_f[2],p_f_f[3],p_f_f[4]] if p_f_f is not None else [0.0]*3 for p_f_f in pfa_filtered[side]],[])
      if all(map(lambda e:e is None,pfa_filtered[side])):
        if not warned[side]:
          CPrint(4,'All points are out of track,',vs_finger.vs_finger,LRToStrS(side))
          warned[side]= True
    return fa
    #fa= []
    #for side in (RIGHT,LEFT):
    ##for side in (LEFT,):
      #fa+= [[p_f_f[2],p_f_f[3],p_f_f[4]] if p_f_f is not None else [0.0]*3 for p_f_f in pfa_filtered[side]]
      #if all(map(lambda e:e is None,pfa_filtered[side])):
        #if not warned[side]:
          #CPrint(4,'All points are out of track,',vs_finger.vs_finger,LRToStrS(side))
          #warned[side]= True
    #return [sum([f[d] for f in fa])/len(fa) for d in range(3)]
  def FNorm(fa):
    f_norm= sum([f*f for f in fa])/(len(fa)/3)
    return f_norm

  wrist= ['wrist_r','wrist_l'][arm]
  x_ext= ct.GetAttr(wrist,'lx')
  xe0= ct.robot.FK(x_ext=x_ext,arm=arm)

  velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)

  #Linear model
  xdim= 3
  #xdim= 1
  StepFilter()
  ydim= len(Farray())
  W= np.mat([[0.0 for c in range(ydim)] for r in range(xdim+1)])
  mbdata= [[],[]]
  diff_max= 0.04
  count= 0

  try:
    while th_info.IsRunning() and not rospy.is_shutdown():
      StepFilter()
      q= ct.robot.Q(arm=arm)
      xe1= ct.robot.FK(q,x_ext=x_ext,arm=arm)

      if count>0 and count%20==0:
        fa= Farray()
        #dp_curr= ToList(MCVec(xe1[:3])-MCVec(xe0[:3]))
        dp_curr= dp
        #dp_curr= [dp_curr[2]]
        print dp, dp_curr, FNorm(fa), FNorm(ToList(W.T.dot(MCVec(AddOne(dp_curr)))))
        UpdateWeightMB(W, dp_curr, fa, mbdata[0], mbdata[1], alpha=max(0.1,1.0/float(count+1)), n_batch=200)
        fp= OpenW('/tmp/fdata.dat','a')
        fp.write('%f %f\n'%(dp_curr[2],FNorm(fa)))
        #fp.write('%f %f\n'%(dp_curr[2],fa[2]))
        fp.close()
        fp= OpenW('/tmp/fmodel.dat')
        for x in np.mgrid[-0.2:0.2:49j]:
          fp.write('%f %f\n'%(x,FNorm(ToList(W.T.dot(MCVec(AddOne([dp_curr[0],dp_curr[1],x])))))))
          #fp.write('%f %f\n'%(x,ToList(W.T.dot(MCVec(AddOne([dp_curr[0],dp_curr[1],x]))))[2]))
        fp.close()

      N= 20
      if count==0:
        dp= [Rand(-0.02,0.02) for d in range(xdim)]
        #dp= [0.0,0.0,dp[0]]
      elif count%N==0:
        #fa= Farray()
        #dp_curr= ToList(MCVec(xe1[:3])-MCVec(xe0[:3]))
        #print dp, dp_curr, FNorm(fa), FNorm(ToList(W.T.dot(MCVec(AddOne(dp_curr)))))
        #UpdateWeight(W, dp_curr, fa, alpha=max(0.1,0.5/float(count+1)))

        dp_curr= ToList(MCVec(xe1[:3])-MCVec(xe0[:3]))
        dp= dp_curr
        #dp= [dp[2]]
        #dp= [dp_curr[2]]
        WWT= W.dot(W.T)
        dp_new= (-la.pinv(WWT[:xdim,:xdim])*WWT[:xdim,xdim]).ravel().tolist()[0]
        print 'opt:',FNorm(ToList(W.T.dot(MCVec(AddOne(dp))))),FNorm(ToList(W.T.dot(MCVec(AddOne(dp_new)))))
        diff= math.sqrt(sum(((x1-x0)**2 for x1,x0 in zip(dp_new,dp))))
        if diff>diff_max:  dp= [x0+(diff_max/diff)*(x1-x0) for x1,x0 in zip(dp_new,dp)]
        else:              dp= dp_new
        #dp= [0.05]*3
        #dp= [0.0,0.0,dp[0]]
      #dp= [0.0,0.0,0.1]

      count+= 1

      J= ct.robot.J(q,arm=arm)
      vq1= ct.robot.limbs[arm].joint_velocities()
      vq1= MCVec([vq1[joint] for joint in ct.robot.JointNames(arm)])
      vx1= J * vq1
      kp= np.diag([0.3,0.3,0.3])
      #kv= np.diag([0.3,0.5,0.5])
      kv= np.diag([0.0,0.0,0.0])
      vx= ToList(kp*(MCVec(xe0[:3])+MCVec(dp)-MCVec(xe1[:3])) - kv*vx1[:3])+[0.0,0.0,0.0]
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
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'vs_maintainf2'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_maintainf2'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    print 'Turn on:','vs_maintainf2'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_maintainf2'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm,ctrl_type))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_maintainf2'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_maintainf2'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_maintainf2'+LRToStrS(RIGHT)
    print 'Turn off:','vs_maintainf2'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_maintainf2'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_maintainf2'+LRToStrS(LEFT))

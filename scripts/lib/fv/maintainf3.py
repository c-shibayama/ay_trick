#!/usr/bin/python
from core_tool import *
def Help():
  return '''Minimizing external force.  Using Baxter's endpoint force estimate, linear regression, and QP.
  Usage:
    fv.maintainf3 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    fv.maintainf3 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.maintainf3
    fv.maintainf3 'clear'
      Stop all threads. Equivalent to:
        fv.maintainf3 'off' LEFT
        fv.maintainf3 'off' RIGHT'''

def WrenchToList2(wrench,l):
  force= wrench['force']
  torque= wrench['torque']
  l[0]= force.x
  l[1]= force.y
  l[2]= force.z
  l[3]= torque.x
  l[4]= torque.y
  l[5]= torque.z

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
  wrench0= [None]
  def GetF():
    wrench= [0.0]*6
    WrenchToList2(ct.robot.limbs[arm].endpoint_effort(), wrench)
    if wrench0[0] is None:
      wrench0[0]= copy.deepcopy(wrench)
    f= [-(fi-fi0) for fi,fi0 in zip(wrench,wrench0[0])]
    return f

  def FNorm(f):
    f_norm= sum([fi*fi for fi in f])/len(f)
    return f_norm

  wrist= ['wrist_r','wrist_l'][arm]
  x_ext= ct.GetAttr(wrist,'lx')
  xe0= ct.robot.FK(x_ext=x_ext,arm=arm)

  velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)

  #Linear model
  xdim= 3
  #xdim= 1
  ydim= len(GetF())
  W= np.mat([[0.0 for c in range(ydim)] for r in range(xdim+1)])
  mbdata= [[],[]]
  diff_max= 0.20
  count= 0

  try:
    while th_info.IsRunning() and not rospy.is_shutdown():
      q= ct.robot.Q(arm=arm)
      xe1= ct.robot.FK(q,x_ext=x_ext,arm=arm)

      if count>0 and count%10==0:
        f= GetF()
        #dp_curr= ToList(MCVec(xe1[:3])-MCVec(xe0[:3]))
        dp_curr= dp
        #dp_curr= [dp_curr[2]]
        print dp, dp_curr, FNorm(f), FNorm(ToList(W.T.dot(MCVec(AddOne(dp_curr)))))
        UpdateWeightMB(W, dp_curr, f, mbdata[0], mbdata[1], alpha=max(0.4,1.0/float(count+1)), n_batch=20)
        fp= OpenW('/tmp/fdata.dat','a')
        fp.write('%f %f\n'%(dp_curr[2],FNorm(f)))
        #fp.write('%f %f\n'%(dp_curr[2],f[2]))
        fp.close()
        fp= OpenW('/tmp/fmodel.dat')
        for x in np.mgrid[-diff_max:diff_max:49j]:
          fp.write('%f %f\n'%(x,FNorm(ToList(W.T.dot(MCVec(AddOne([dp_curr[0],dp_curr[1],x])))))))
          #fp.write('%f %f\n'%(x,ToList(W.T.dot(MCVec(AddOne([dp_curr[0],dp_curr[1],x]))))[2]))
        fp.close()

      N= 10
      if count==0:
        dp= [Rand(-0.02,0.02) for d in range(xdim)]
        #dp= [0.0,0.0,dp[0]]
      elif count%N==0:
        #f= GetF()
        #dp_curr= ToList(MCVec(xe1[:3])-MCVec(xe0[:3]))
        #print dp, dp_curr, FNorm(f), FNorm(ToList(W.T.dot(MCVec(AddOne(dp_curr)))))
        #UpdateWeight(W, dp_curr, f, alpha=max(0.1,0.5/float(count+1)))

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

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'vs_maintainf3'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_maintainf3'+LRToStrS(arm),'is already on'

    print 'Turn on:','vs_maintainf3'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_maintainf3'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm,ctrl_type))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_maintainf3'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_maintainf3'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_maintainf3'+LRToStrS(RIGHT)
    print 'Turn off:','vs_maintainf3'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_maintainf3'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_maintainf3'+LRToStrS(LEFT))

#!/usr/bin/python
from core_tool import *
import baxter_core_msgs
def Help():
  return '''Track an external force.  Using Baxter's endpoint force estimate, and torque control.
    !!WARNING!! This controller sometimes gets unstable!!
  Usage:
    bx.test.bxtrackf2 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    bx.test.bxtrackf2 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    bx.test.bxtrackf2
    bx.test.bxtrackf2 'clear'
      Stop all threads. Equivalent to:
        bx.test.bxtrackf2 'off' LEFT
        bx.test.bxtrackf2 'off' RIGHT'''

#def WrenchToList(wrench,l):
  #l[0]= wrench.force.x
  #l[1]= wrench.force.y
  #l[2]= wrench.force.z
  #l[3]= wrench.torque.x
  #l[4]= wrench.torque.y
  #l[5]= wrench.torque.z

def WrenchToList2(wrench,l):
  force= wrench['force']
  torque= wrench['torque']
  l[0]= force.x
  l[1]= force.y
  l[2]= force.z
  l[3]= torque.x
  l[4]= torque.y
  l[5]= torque.z

#def ListToWrench(l,wrench):
  #wrench.force.x= l[0]
  #wrench.force.y= l[1]
  #wrench.force.z= l[2]
  #wrench.torque.x= l[3]
  #wrench.torque.y= l[4]
  #wrench.torque.z= l[5]

#def Callback(ct,msg,arm):
  #lwrench= [0]*6
  #WrenchToList(msg.wrench, lwrench)
  #ct.GetAttr(TMP,'bxwrench')[arm]= lwrench

def TrackingLoop(th_info, ct, arm, ctrl_type):
  #wrench= ct.GetAttr(TMP,'bxwrench')
  wrench= [0.0]*6

  #while th_info.IsRunning() and not rospy.is_shutdown():
    #if len(wrench[arm])>0:  break
    #rospy.sleep(0.01)

  f0= None

  tqctrl= ct.Load('bx.tqctrl').TTqCtrl(ct,arm=arm)
  #kp= [20.0, 20.0, 20.0, 20.0, 10.0, 15.0, 5.0]
  #kd= [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
  kp= [40.0, 40.0, 40.0, 40.0, 20.0, 30.0, 10.0]
  kd= [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
  kp= [3.0*k for k in kp]
  kd= [3.0*k for k in kd]
  q_trg= ct.robot.Q(arm=arm)
  time0= rospy.Time.now()

  try:
    wrist= ['wrist_r','wrist_l'][arm]
    while th_info.IsRunning() and not rospy.is_shutdown():
      active= False
      if (rospy.Time.now()-time0).to_sec()>2.0:
        WrenchToList2(ct.robot.limbs[arm].endpoint_effort(), wrench)
        if f0 is None:
          f0= Vec(copy.deepcopy(wrench))
        f_diff= Vec(wrench) - f0

        fd_l= -f_diff
        #print fd_l,f0
        #print ' '.join(map(lambda f:'%0.3f'%f,fd_l))
        f_gain= 0.01
        t_gain= 0.1
        if ctrl_type=='position':
          fd= ToList(f_gain*fd_l[:3]) + [0.0,0.0,0.0]
        elif ctrl_type=='pose':
          fd= ToList(f_gain*fd_l[:3]) + ToList(t_gain*fd_l[3:])
        elif ctrl_type=='orientation':
          fd= [0.0,0.0,0.0] + ToList(t_gain*fd_l[3:])
        active= True
      else:
        fd= [0.0]*6
      #fd= [0.0]*6  #WARNING: Be careful to comment this!!!

      q= ct.robot.Q(arm=arm)
      J= ct.robot.J(q,arm=arm)
      vq= ct.robot.limbs[arm].joint_velocities()
      vq= [vq[joint] for joint in ct.robot.JointNames(arm)]
      vx= 0.001*MCVec(fd)
      #alpha= 0.5
      #q_trg= [(1.0-alpha)*q1+alpha*q0 if abs(q0-q1)>0.1 else q1 for q1,q0 in zip(q_trg,q)]
      #print ' '.join(map(lambda f:'%0.3f'%f,la.pinv(J)*vx))
      q_trg= ToList(MCVec(q_trg)+la.pinv(J)*vx)
      tq= [kp[j]*(q_trg[j]-q[j])-kd[j]*vq[j] for j in range(7)]
      tqctrl.Step(tq)

  finally:
    tqctrl.Finish()
    ct.thread_manager.Stop(name='bxtrackf2'+LRToStrS(arm))

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'bxtrackf2'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'bxtrackf2'+LRToStrS(arm),'is already on'

    #ct.SetAttr(TMP,'bxwrench', [[],[]])
    #ct.AddSub('estate2_'+LRToStrS(arm), '/robot/limb/%s/endpoint_state'%LRTostr(arm), baxter_core_msgs.msg.EndpointState,
             #lambda msg,ct=ct,arm=arm: Callback(ct,msg,arm))

    print 'Turn on:','bxtrackf2'+LRToStrS(arm)
    ct.thread_manager.Add(name='bxtrackf2'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm,ctrl_type))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','bxtrackf2'+LRToStrS(arm)
    ct.thread_manager.Stop(name='bxtrackf2'+LRToStrS(arm))
    #ct.DelSub('estate2_'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','bxtrackf2'+LRToStrS(RIGHT)
    print 'Turn off:','bxtrackf2'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='bxtrackf2'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='bxtrackf2'+LRToStrS(LEFT))
    #ct.DelSub('estate2_'+LRToStrS(RIGHT))
    #ct.DelSub('estate2_'+LRToStrS(LEFT))

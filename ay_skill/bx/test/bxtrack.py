#!/usr/bin/python
from core_tool import *
import baxter_core_msgs
def Help():
  return '''Track human guide.  Using torque control.
  Usage:
    bx.test.bxtrack 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    bx.test.bxtrack 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    bx.test.bxtrack
    bx.test.bxtrack 'clear'
      Stop all threads. Equivalent to:
        bx.test.bxtrack 'off' LEFT
        bx.test.bxtrack 'off' RIGHT'''

def TrackingLoop(th_info, ct, arm, ctrl_type):

  tqctrl= ct.Load('bx.tqctrl').TTqCtrl(arm,ct)
  #kp= [20.0, 20.0, 20.0, 20.0, 10.0, 15.0, 5.0]
  #kd= [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
  kp= [40.0, 60.0, 40.0, 40.0, 30.0, 20.0, 1.0]
  kd= [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
  kp= [3.0*k for k in kp]
  kd= [3.0*k for k in kd]
  q_trg= ct.robot.Q(arm=arm)
  time0= rospy.Time.now()

  try:
    wrist= ['wrist_r','wrist_l'][arm]
    while th_info.IsRunning() and not rospy.is_shutdown():
      q= ct.robot.Q(arm=arm)
      J= ct.robot.J(q,arm=arm)
      vq= ct.robot.limbs[arm].joint_velocities()
      vq= [vq[joint] for joint in ct.robot.JointNames(arm)]
      alpha= 0.3
      q_th= 0.03
      q_trg= [(1.0-alpha)*q1+alpha*q0 if abs(q0-q1)>q_th else q1 for q1,q0 in zip(q_trg,q)]
      tq= [kp[j]*(q_trg[j]-q[j])-kd[j]*vq[j] for j in range(7)]
      tqctrl.Step(tq)

  finally:
    tqctrl.Finish()
    ct.thread_manager.Stop(name='bxtrack'+LRToStrS(arm))

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'bxtrack'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'bxtrack'+LRToStrS(arm),'is already on'

    print 'Turn on:','bxtrack'+LRToStrS(arm)
    ct.thread_manager.Add(name='bxtrack'+LRToStrS(arm), target=lambda th_info: TrackingLoop(th_info,ct,arm,ctrl_type))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','bxtrack'+LRToStrS(arm)
    ct.thread_manager.Stop(name='bxtrack'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','bxtrack'+LRToStrS(RIGHT)
    print 'Turn off:','bxtrack'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='bxtrack'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='bxtrack'+LRToStrS(LEFT))

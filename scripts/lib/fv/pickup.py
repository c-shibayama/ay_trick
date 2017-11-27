#!/usr/bin/python
from core_tool import *
def Help():
  return '''Slip-based picking up.
    Similar to vs_hold, but the motion is automated.
  Usage:
    fv.pickup 'on' [, ARM]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.pickup 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.pickup
    fv.pickup 'clear'
      Stop all threads. Equivalent to:
        fv.pickup 'off' LEFT
        fv.pickup 'off' RIGHT'''
def PickupLoop(th_info, ct, arm):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  ct.Run('fv.finger3','stop_detect_obj',arm)

  #g_pos= ct.robot.GripperPos(arm)
  #while th_info.IsRunning() and not rospy.is_shutdown():
    #if sum(vs_finger.mv_s[0])+sum(vs_finger.mv_s[1])>0.1:
      #g_pos-= 0.0005 if arm==LEFT else 0.002
      ##ct.robot.MoveGripper(pos=g_pos, arm=arm, speed=100.0, blocking=False)
      ##rospy.sleep(0.001)
      ##g_pos= ct.robot.GripperPos(arm)
      #ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=1.0, speed=1.0, blocking=False)
      #for i in range(100):
        #if abs(ct.robot.GripperPos(arm)-g_pos)<0.0002:  break
        #rospy.sleep(0.0001)
      #g_pos= ct.robot.GripperPos(arm)
    #else:
      #rospy.sleep(0.001)
    #rospy.sleep(0.02)

  x0= ct.robot.FK(arm=arm)
  tm0= rospy.Time.now()
  g_pos= ct.robot.GripperPos(arm)
  g_motion= None
  slip_detected= False
  z_offset= 0.0

  velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)
  try:
    while th_info.IsRunning() and not rospy.is_shutdown():
      if sum(vs_finger.mv_s[0])+sum(vs_finger.mv_s[1])>0.1:
        slip_detected= True

      if g_motion is None:
        if slip_detected:
          #g_pos-= 0.0005 if arm==LEFT else 0.002
          g_pos-= 0.0005 if arm==LEFT else 0.003
          #ct.robot.MoveGripper(pos=g_pos, arm=arm, speed=100.0, blocking=False)
          #rospy.sleep(0.001)
          #g_pos= ct.robot.GripperPos(arm)
          ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=1.0, speed=1.0, blocking=False)
          g_motion= 100
          slip_detected= False
      else:
        if g_motion>0:
          if abs(ct.robot.GripperPos(arm)-g_pos)<0.0002:  g_motion= 0
          else:  g_motion-= 1
        if g_motion<=0:
          g_motion-= 1
          if g_motion<-20:
            g_pos= ct.robot.GripperPos(arm)
            g_motion= None


      vz_up= 0.01
      vz_down= -0.1
      tm= (rospy.Time.now()-tm0).to_sec()
      #if tm>5.0: break
      if g_motion is None and not slip_detected:
        #print 'No slip',rospy.Time.now()
        z_offset= min(0.10, z_offset + vz_up*velctrl.TimeStep())
      else:
        z_offset= max(-0.01, z_offset + vz_down*velctrl.TimeStep())

      x1= ct.robot.FK(arm=arm)
      q= ct.robot.Q(arm=arm)
      J= ct.robot.J(q,arm=arm)
      vq1= ct.robot.limbs[arm].joint_velocities()
      vq1= MCVec([vq1[joint] for joint in ct.robot.JointNames(arm)])
      vx1= J * vq1

      amp= 0.02 if z_offset<0.04 else 0.0
      omega= 4.0
      kp= 0.8
      z_trg= z_offset+x0[2]+amp*0.5*(1.0-math.cos(omega*tm))
      ##kv= np.diag([0.3,0.5,0.5])
      ##vx= ToList(MCVec(vp) - kv*vx0[:3])+[0.0,0.0,0.0]
      #vx= [0.0,0.0, 0.005+0.02*math.cos(5.0*tm), 0.0,0.0,0.0]
      vx= [0.0,0.0, kp*(z_trg-x1[2]), 0.0,0.0,0.0]

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
    if 'vs_pickup'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_pickup'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    print 'Turn on:','vs_pickup'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_pickup'+LRToStrS(arm), target=lambda th_info: PickupLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_pickup'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_pickup'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_pickup'+LRToStrS(RIGHT)
    print 'Turn off:','vs_pickup'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_pickup'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_pickup'+LRToStrS(LEFT))



#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of slip-based holding.
    Closing gripper if slip is detected.
  Usage:
    fv.vs_hold 'on' [, ARM]
      Turn on a holding thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vs_hold 'off' [, ARM]
      Stop a holding thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vs_hold
    fv.vs_hold 'clear'
      Stop all threads. Equivalent to:
        fv.vs_hold 'off' LEFT
        fv.vs_hold 'off' RIGHT'''
def HoldLoop(th_info, ct, arm):
  ct.Run('fv.ctrl_params')
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  ct.Run('fv.vs_finger3','stop_detect_obj',arm)

  g_pos= ct.robot.GripperPos(arm)
  while th_info.IsRunning() and not rospy.is_shutdown():
    if sum(vs_finger.mv_s[0])+sum(vs_finger.mv_s[1])>0.06: #0.1
      print 'slip',rospy.Time.now()
      #g_pos-= 0.0005 if arm==LEFT else 0.002
      g_pos-= ct.GetAttr('fv_ctrl','min_gstep')[arm]
      #ct.robot.MoveGripper(pos=g_pos, arm=arm, speed=100.0, blocking=False)
      #rospy.sleep(0.001)
      #g_pos= ct.robot.GripperPos(arm)
      ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=1.0, speed=1.0, blocking=False)
      for i in range(100):  #100
        if abs(ct.robot.GripperPos(arm)-g_pos)<0.5*ct.GetAttr('fv_ctrl','min_gstep')[arm]:  break
        rospy.sleep(0.0001)
      g_pos= ct.robot.GripperPos(arm)
    else:
      rospy.sleep(0.001)
    rospy.sleep(0.04)  #0.02

  #print 'Open the gripper and start object detection?'
  #if ct.AskYesNo():
    #ct.robot.OpenGripper(arm)
    ##Start object detection
    #ct.Run('fv.vs_finger3','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    if 'vs_hold'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_hold'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.vs_finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.vs_finger3','setup',arm)

    print 'Turn on:','vs_hold'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_hold'+LRToStrS(arm), target=lambda th_info: HoldLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_hold'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_hold'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_hold'+LRToStrS(RIGHT)
    print 'Turn off:','vs_hold'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_hold'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_hold'+LRToStrS(LEFT))



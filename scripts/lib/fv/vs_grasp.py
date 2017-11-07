#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of force-based grasping.
    Closing gripper until it finds a force.
  Usage:
    fv.vs_grasp 'on' [, ARM]
      Turn on a grasping thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vs_grasp 'off' [, ARM]
      Stop a grasping thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vs_grasp
    fv.vs_grasp 'clear'
      Stop all threads. Equivalent to:
        fv.vs_grasp 'off' LEFT
        fv.vs_grasp 'off' RIGHT'''
def GraspLoop(th_info, ct, arm):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))
  #fa0= vs_finger.force_array[arm]
  #n_change= lambda: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(vs_finger.force_array[arm],fa0)])
  #dth= 7
  dth= max(vs_finger.dstate[0],vs_finger.dstate[1]) + 4
  continue_cond= lambda: vs_finger.dstate[0]<dth and vs_finger.dstate[1]<dth
  #continue_cond= lambda: n_change()<5

  #Stop object detection
  ct.Run('fv.vs_finger3','stop_detect_obj',arm)

  #g_pos= ct.robot.GripperPos(arm)
  #if arm==RIGHT:
    #while g_pos>0.0 and continue_cond():
      #ct.robot.MoveGripper(pos=g_pos, arm=arm, speed=10.0, blocking=True)
      #g_pos-= 0.001
  #elif arm==LEFT:
    #ct.robot.MoveGripper(pos=0.0, max_effort=1.0, speed=1.0, arm=arm)
    #while ct.robot.GripperPos(arm)>0.001 and continue_cond():
      #rospy.sleep(0.001)
    #ct.robot.grippers[LEFT].StopGripper()

  g_pos= ct.robot.GripperPos(arm)
  while th_info.IsRunning() and not rospy.is_shutdown():
    if g_pos<0.001 or not continue_cond():
      print 'Done'
      break

    g_pos-= 0.002 if isinstance(ct.robot.grippers[arm], baxter_interface.Gripper) else 0.0005
    ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=1.0, speed=1.0, blocking=False)
    for i in range(100):
      if abs(ct.robot.GripperPos(arm)-g_pos)<0.0002:  break
      rospy.sleep(0.0001)
    #rospy.sleep(0.1)
    g_pos= ct.robot.GripperPos(arm)

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
    if 'vs_grasp'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_grasp'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.vs_finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.vs_finger3','setup',arm)

    print 'Turn on:','vs_grasp'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_grasp'+LRToStrS(arm), target=lambda th_info: GraspLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_grasp'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_grasp'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_grasp'+LRToStrS(RIGHT)
    print 'Turn off:','vs_grasp'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_grasp'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_grasp'+LRToStrS(LEFT))


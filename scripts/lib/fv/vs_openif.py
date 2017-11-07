#!/usr/bin/python
from core_tool import *
def Help():
  return '''Opening gripper if an external force is applied.
  Usage:
    fv.vs_openif 'on' [, ARM]
      Turn on an opening thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vs_openif 'off' [, ARM]
      Stop an opening thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vs_openif
    fv.vs_openif 'clear'
      Stop all threads. Equivalent to:
        fv.vs_openif 'off' LEFT
        fv.vs_openif 'off' RIGHT'''
def OpeningLoop(th_info, ct, arm):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #Stop object detection
  ct.Run('fv.vs_finger3','stop_detect_obj',arm)

  fa0= copy.deepcopy(vs_finger.force_array)
  #FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
  n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(vs_finger.force_array[side],fa0[side])])
  #dth= 5

  while th_info.IsRunning() and not rospy.is_shutdown():
    if n_change(0)+n_change(1)>7:
      print 'Force is applied'
      ct.robot.OpenGripper(arm)
      break
    elif sum(vs_finger.mv_s[0])+sum(vs_finger.mv_s[1])>0.3:
      print 'Slip is detected'
      ct.robot.OpenGripper(arm)
      break
    else:
      rospy.sleep(0.02)

  ct.Run('fv.vs_finger3','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    if 'vs_openif'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_openif'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.vs_finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.vs_finger3','setup',arm)

    ct.Run('fv.vs_grasp','off',arm)
    ct.Run('fv.vs_hold','off',arm)

    print 'Turn on:','vs_openif'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_openif'+LRToStrS(arm), target=lambda th_info: OpeningLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_openif'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_openif'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_openif'+LRToStrS(RIGHT)
    print 'Turn off:','vs_openif'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_openif'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_openif'+LRToStrS(LEFT))




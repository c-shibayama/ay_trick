#!/usr/bin/python
from core_tool import *
def Help():
  return '''Opening gripper if an external force is applied.
  Usage:
    fv.openif 'on' [, ARM]
      Turn on an opening thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      OPTIONS: Options of OpeningLoop (see OpeningLoopDefaultOptions).
    fv.openif 'off' [, ARM]
      Stop an opening thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.openif
    fv.openif 'clear'
      Stop all threads. Equivalent to:
        fv.openif 'off' LEFT
        fv.openif 'off' RIGHT'''

def OpeningLoopDefaultOptions():
  return {
    'slip_threshold': 0.3,  #Threshold of slip amount to open the gripper.
    'nforce_threshold': 7,  #Threshold of number of force changing points to open the gripper.
    'dw_grip': 0.02,  #Displacement of gripper movement.
    }

def OpeningLoop(th_info, ct, arm, options):
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))

  #Stop object detection
  ct.Run('fv.fv','stop_detect_obj',arm)

  fa0= copy.deepcopy(fv_data.force_array)
  #FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
  n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(fv_data.force_array[side],fa0[side])])
  #dth= 5

  while th_info.IsRunning() and not rospy.is_shutdown():
    if n_change(0)+n_change(1)>options['nforce_threshold']:
      print 'Force is applied'
      #ct.robot.OpenGripper(arm)
      ct.robot.MoveGripper(pos=ct.robot.GripperPos(arm)+options['dw_grip'], arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm])
      break
    elif sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>options['slip_threshold']:
      print 'Slip is detected'
      #ct.robot.OpenGripper(arm)
      ct.robot.MoveGripper(pos=ct.robot.GripperPos(arm)+options['dw_grip'], arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm])
      break
    else:
      rospy.sleep(0.02)

  ct.Run('fv.fv','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    user_options= args[1] if len(args)>1 else {}

    options= OpeningLoopDefaultOptions()
    InsertDict(options, user_options)

    if 'vs_openif'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_openif'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    print 'Turn on:','vs_openif'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_openif'+LRToStrS(arm), target=lambda th_info: OpeningLoop(th_info,ct,arm,options))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_openif'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_openif'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_openif'+LRToStrS(RIGHT)
    print 'Turn off:','vs_openif'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_openif'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_openif'+LRToStrS(LEFT))




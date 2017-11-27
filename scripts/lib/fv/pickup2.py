#!/usr/bin/python
from core_tool import *
def Help():
  return '''Slip-based picking up version 2.
    Similar to vs_hold, but the motion is automated.
  Usage:
    fv.pickup2 'on' [, ARM [, OPTIONS]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      OPTIONS: Options of PickupLoop (see PickupLoopDefaultOptions).
    fv.pickup2 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.pickup2
    fv.pickup2 'clear'
      Stop all threads. Equivalent to:
        fv.pickup2 'off' LEFT
        fv.pickup2 'off' RIGHT
    fv.pickup2 'once' [, ARM [, OPTIONS]]
      Execute motion once (not thread).
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      OPTIONS: Options of PickupLoop (see PickupLoopDefaultOptions).
'''

def PickupLoopDefaultOptions():
  return {
    'slip_sensitivity': 0.6,  #Slip sensitivity.
    'z_final': 0.15,  #Final height (offset from the beginning).
    'auto_stop': False,  #Whether automatically stops when pickup is completed.
    'time_out': None,  #Timeout in seconds, or no timeout (None).
    'stop_velctrl': True,  #Exit velocity control mode after execution.
    'resume_detect_obj': True,  #Restart object detection after execution.
    'bring_up_after_exit': False,  #Even if an exit condition is satisfied, robot brings up an object to target height.
    'log': {},  #[output] Execution results are stored into this dictionary.
    }

def PickupLoop(th_info, ct, arm, options=PickupLoopDefaultOptions()):
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))

  #get_center= lambda: [0.5*(vs_finger.obj_center[0][0]-vs_finger.obj_center[1][0]),
                       #0.5*(vs_finger.obj_center[0][1]+vs_finger.obj_center[1][1]) ]
  #def get_center():
    #xy= [[],[]]
    #area_min= 0.1
    #for side in range(2):
      #if vs_finger.obj_area[side]>area_min:
        #xy[0].append(vs_finger.obj_center[side][0])
        #xy[1].append(vs_finger.obj_center[side][1])
    #if len(xy[0])==0:  return [0.0,0.0]
    #return [sum(xy[0])/float(len(xy[0])), sum(xy[1])/float(len(xy[1]))]

  #Get object center computed as a sum of object centers from both fingers
  #weighted with their areas.
  def get_center_avr():
    sum_area= sum(vs_finger.obj_area)
    if sum_area<0.02:  return [0.0,0.0]
    w= [vs_finger.obj_area[0]/sum_area, vs_finger.obj_area[1]/sum_area]
    return [w[0]*vs_finger.obj_center[0][0] - w[1]*vs_finger.obj_center[1][0],
            w[0]*vs_finger.obj_center[0][1] + w[1]*vs_finger.obj_center[1][1] ]

  obj_area0= [max(0.1,area) for area in vs_finger.obj_area]

  #center_set= [get_center()]
  get_slip1= lambda: sum(vs_finger.mv_s[0])+sum(vs_finger.mv_s[1])
  get_slip1_nml= lambda: sum(vs_finger.mv_s[0])/obj_area0[0]+sum(vs_finger.mv_s[1])/obj_area0[1]
  #def get_slip2():
    #center= get_center()
    #center_set.append(center)
    #if len(center_set)>2000:  center_set.pop(0)
    #return Dist(center,center_set[0])

  #Stop object detection
  ct.Run('fv.finger3','stop_detect_obj',arm)

  l= TContainer(debug=True)
  l.x0= ct.robot.FK(arm=arm)
  l.ez0=  RotToExyz(QToRot(Transform(l.x0,ct.GetAttr('wrist_'+LRToStrs(arm),'lx'))[3:]))[0]
  l.x0[2]+= l.ez0[2]*ct.robot.FingertipOffset(arm=arm)
  l.tm0= rospy.Time.now()
  l.g_pos= ct.robot.GripperPos(arm)
  #g_motion= None
  #slip_detected= False
  l.z_offset= 0.0
  l.slip_amt= 0.0
  l.slip_detected= False
  l.z_err= None
  l.g_motion= 0  #If >0, gripper is in motion.
  l.log= options['log']
  l.log['area']= []
  l.log['slip']= []
  l.log['slip_nml']= []
  l.log['center']= []
  l.log['f']= []
  l.log['g_pos']= []
  l.log['z_trg']= []
  l.log['x']= []

  l.velctrl= ct.Load('bx.velctrl').TVelCtrl(ct,arm=arm)
  l.ctrl_step= 0  #Counter for logging cycle control.

  def SetZOffset(z_offset):
    l.z_offset= z_offset
    l.z_err= None

  def ZTrgErr():
    x1= ct.robot.FK(arm=arm)
    l.z_trg= l.x0[2] + l.z_offset - l.ez0[2]*ct.robot.FingertipOffset(arm=arm)
    l.z_err= l.z_trg-x1[2]
    return l.z_trg, l.z_err

  def ResetSlipDetect():
    l.slip_amt= 0.0
    l.slip_detected= False

  def GraspMore():
    count_wait_gmove= 50
    #l.g_pos-= 0.0005 if arm==LEFT else 0.002
    l.g_pos-= 0.0007 if arm==LEFT else 0.003
    #ct.robot.MoveGripper(pos=l.g_pos, arm=arm, speed=100.0, blocking=False)
    #rospy.sleep(0.001)
    #l.g_pos= ct.robot.GripperPos(arm)
    ct.robot.MoveGripper(pos=l.g_pos, arm=arm, max_effort=1.0, speed=1.0, blocking=False)
    l.g_motion= count_wait_gmove

  def ControlStep():
    #slip1= get_slip1()
    ##slip2= get_slip2()
    ##if slip2<0.1:  slip2= 0.0
    #l.slip_curr= slip1 # + slip2
    ##print get_slip1(),get_slip2()
    ##l.slip_amt+= l.slip_curr*l.velctrl.TimeStep()
    ##if l.slip_amt>0.05:
      ##l.slip_detected= True

    if l.ctrl_step%50==0:
      tm= rospy.Time.now().to_sec()
      l.log['area'].append([tm,sum(vs_finger.obj_area)])
      l.log['slip'].append([tm,get_slip1()])
      l.log['slip_nml'].append([tm,get_slip1_nml()])
      l.log['center'].append([tm,get_center_avr()])
      l.log['f'].append([tm,map(float,vs_finger.force[0][:3])+map(float,vs_finger.force[1][:3])])
      l.log['g_pos'].append([tm,ct.robot.GripperPos(arm)])
      l.log['z_trg'].append([tm,float(ZTrgErr()[0])])
      l.log['x'].append([tm,ToList(ct.robot.FK(arm=arm))])

    #l.slip_curr= get_slip1()
    #if l.slip_curr>0.1:
    l.slip_curr= get_slip1_nml()  #Normalized by object area.
    if l.slip_curr>options['slip_sensitivity']:  #NOTE: Slip sensitivity to be tuned.
      #print get_slip1(), get_slip1_nml()
      l.slip_detected= True
      if l.g_motion==0:
        GraspMore()

    if l.g_motion>0:
      if abs(ct.robot.GripperPos(arm)-l.g_pos)<0.0002:  l.g_motion= 0
      else:  l.g_motion-= 1

    x1= ct.robot.FK(arm=arm)
    q= ct.robot.Q(arm=arm)
    J= ct.robot.J(q,arm=arm)
    vq1= ct.robot.limbs[arm].joint_velocities()
    vq1= MCVec([vq1[joint] for joint in ct.robot.JointNames(arm)])
    vx1= J * vq1

    #amp= 0.02 if z_offset<0.04 else 0.0
    #omega= 2.0
    kp= [1.0,1.0, 10.0,  1.0,1.0,1.0]
    kv= [0.1,0.1, 2.5,  0.1,0.1,0.1]
    ##kv= np.diag([0.3,0.5,0.5])
    ##vx= ToList(MCVec(vp) - kv*vx0[:3])+[0.0,0.0,0.0]
    #vx= [0.0,0.0, 0.005+0.02*math.cos(5.0*tm), 0.0,0.0,0.0]
    l.z_trg,l.z_err= ZTrgErr()
    z_err_max= 0.006
    if l.z_err>z_err_max:  z_err= z_err_max
    elif l.z_err<-z_err_max:  z_err= -z_err_max
    else:  z_err= l.z_err
    #vx= [0.0,0.0, kp*z_err-kv*vx1[2,0], 0.0,0.0,0.0]
    vx= [kp[d]*(l.x0[d]-x1[d]) - kv[d]*vx1[d,0] for d in range(6)]
    vx[2]= kp[2]*z_err-kv[2]*vx1[2,0]
    #print vx

    dq= ToList(la.pinv(J)*MCVec(vx))
    l.velctrl.Step(dq)
    l.ctrl_step+= 1

  sm= TStateMachine(debug=False, local_obj=l)
  sm.EventCallback= ct.SMCallback
  sm.StartState= 'bring_test'

  time_out= options['time_out']
  action_quit= TFSMConditionedAction()
  action_quit.Condition= lambda: any((rospy.is_shutdown(),
                                      not th_info.IsRunning() if th_info is not None else False,
                                      (rospy.Time.now()-l.tm0).to_sec()>time_out if time_out is not None else False))
  action_quit.NextState= 'bring_up_to_exit' if options['bring_up_after_exit'] else EXIT_STATE

  action_ctrlstep= TFSMConditionedAction()
  action_ctrlstep.Condition= lambda: True
  action_ctrlstep.Action= ControlStep
  action_ctrlstep.NextState= ORIGIN_STATE

  sm.NewState('bring_test')
  sm['bring_test'].EntryAction= lambda: (SetZOffset(0.005), ResetSlipDetect())
  sm['bring_test'].NewAction()
  sm['bring_test'].Actions[-1]= action_quit
  sm['bring_test'].NewAction()
  sm['bring_test'].Actions[-1].Condition= lambda: l.slip_detected
  sm['bring_test'].Actions[-1].Action= ResetSlipDetect
  sm['bring_test'].Actions[-1].NextState= 'to_initial'
  sm['bring_test'].NewAction()
  sm['bring_test'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.002
  sm['bring_test'].Actions[-1].NextState= 'wait'
  sm['bring_test'].ElseAction= action_ctrlstep

  sm.NewState('to_initial')
  sm['to_initial'].EntryAction= lambda: SetZOffset(0.0)
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= action_quit
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.002
  sm['to_initial'].Actions[-1].NextState= 'grasp_more'
  sm['to_initial'].ElseAction= action_ctrlstep

  sm.NewState('grasp_more')
  sm['grasp_more'].EntryAction= GraspMore
  sm['grasp_more'].NewAction()
  sm['grasp_more'].Actions[-1]= action_quit
  sm['grasp_more'].NewAction()
  sm['grasp_more'].Actions[-1].Condition= lambda: l.g_motion==0
  sm['grasp_more'].Actions[-1].NextState= 'bring_test'
  sm['grasp_more'].ElseAction= action_ctrlstep

  sm.NewState('wait')
  sm['wait'].EntryAction= lambda: setattr(l,'tm1',rospy.Time.now()+rospy.Duration(0.4))
  sm['wait'].NewAction()
  sm['wait'].Actions[-1]= action_quit
  sm['wait'].NewAction()
  sm['wait'].Actions[-1].Condition= lambda: l.slip_detected
  sm['wait'].Actions[-1].Action= ResetSlipDetect
  sm['wait'].Actions[-1].NextState= 'to_initial'
  sm['wait'].NewAction()
  sm['wait'].Actions[-1].Condition= lambda: l.tm1>rospy.Time.now()  #WARNING:BUG: l.tm1<rospy.Time.now()
  sm['wait'].Actions[-1].Action= GraspMore  #NEW
  sm['wait'].Actions[-1].NextState= 'bring_up'
  sm['wait'].ElseAction= action_ctrlstep

  sm.NewState('bring_up')
  sm['bring_up'].EntryAction= lambda: SetZOffset(options['z_final'])
  sm['bring_up'].NewAction()
  sm['bring_up'].Actions[-1]= action_quit
  sm['bring_up'].NewAction()
  sm['bring_up'].Actions[-1].Condition= lambda: l.slip_detected
  sm['bring_up'].Actions[-1].Action= ResetSlipDetect
  sm['bring_up'].Actions[-1].NextState= 'to_initial'
  if options['auto_stop']:
    sm['bring_up'].NewAction()
    sm['bring_up'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.005
    sm['bring_up'].Actions[-1].NextState= EXIT_STATE
  sm['bring_up'].ElseAction= action_ctrlstep

  sm.NewState('bring_up_to_exit')
  sm['bring_up_to_exit'].EntryAction= lambda: SetZOffset(options['z_final'])
  sm['bring_up_to_exit'].NewAction()
  sm['bring_up_to_exit'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.005
  sm['bring_up_to_exit'].Actions[-1].NextState= EXIT_STATE
  sm['bring_up_to_exit'].ElseAction= action_ctrlstep

  try:
    ct.RunSM(sm,'vs_pickup2')

  finally:
    if options['stop_velctrl']:  l.velctrl.Finish()
    if options['resume_detect_obj']: ct.Run('fv.finger3','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    user_options= args[1] if len(args)>1 else {}

    options= PickupLoopDefaultOptions()
    InsertDict(options, user_options)
    if 'log' in user_options:  options['log']= user_options['log']

    if 'vs_pickup2'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_pickup2'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    print 'Turn on:','vs_pickup2'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_pickup2'+LRToStrS(arm), target=lambda th_info: PickupLoop(th_info,ct,arm,options))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_pickup2'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_pickup2'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_pickup2'+LRToStrS(RIGHT)
    print 'Turn off:','vs_pickup2'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_pickup2'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_pickup2'+LRToStrS(LEFT))

  elif command=='once':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    user_options= args[1] if len(args)>1 else {}

    options= PickupLoopDefaultOptions()
    InsertDict(options, user_options)
    if 'log' in user_options:  options['log']= user_options['log']
    options['auto_stop']= True

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    PickupLoop(None, ct, arm, options=options)


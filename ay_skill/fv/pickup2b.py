#!/usr/bin/python
from core_tool import *
def Help():
  return '''Slip-based picking up version 2b.
    Similar to vs_hold, but the motion is automated.
  Usage:
    fv.pickup2b 'on' [, ARM [, OPTIONS]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      OPTIONS: Options of PickupLoop (see PickupLoopDefaultOptions).
    fv.pickup2b 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.pickup2b
    fv.pickup2b 'clear'
      Stop all threads. Equivalent to:
        fv.pickup2b 'off' LEFT
        fv.pickup2b 'off' RIGHT
    fv.pickup2b 'once' [, ARM [, OPTIONS]]
      Execute motion once (not thread).
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      OPTIONS: Options of PickupLoop (see PickupLoopDefaultOptions).
'''

def PickupLoopDefaultOptions(ct):
  ct.Run('fv.ctrl_params')
  return {
    'sensitivity_slip': 0.06,  #Sensitivity of slip detection (smaller is more sensitive).
    'sensitivity_oc':0.2,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
    'sensitivity_oo':0.5,  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
    'sensitivity_oa':0.3,  #Sensitivity of object-area-change detection (smaller is more sensitive).
    #'area_drop_rate': 0.8,
    'area_drop_rate': 0.6,  #If object area becomes smaller than this rate, it's considered as dropped.
    'z_final': ct.GetAttr('fv_ctrl','pickup2a_z_final'),  #Final height (offset from the beginning).
    'obj_area_filter_len': 5,  #Filter length for obj_area.
    'auto_stop': False,  #Whether automatically stops when pickup is completed.
    'time_out': None,  #Timeout in seconds, or no timeout (None).
    'resume_detect_obj': False,  #Restart object detection after execution.
    'bring_up_after_exit': False,  #Even if an exit condition is satisfied, robot brings up an object to target height.
    'keep_thread_after_exit': False,  #Keep PickupLoop thread after everything has finished.
    'log': {},  #[output] Execution results are stored into this dictionary.
    }

def PickupLoop(th_info, ct, arm, options):
  ct.Run('fv.ctrl_params')
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))

  #Get object center computed as a sum of object centers from both fingers
  #weighted with their areas.
  def get_center_avr():
    sum_area= sum(fv_data.obj_area_filtered)
    if sum_area<0.02:  return [0.0,0.0]
    w= [fv_data.obj_area_filtered[0]/sum_area, fv_data.obj_area_filtered[1]/sum_area]
    return [w[0]*fv_data.obj_center[0][0] - w[1]*fv_data.obj_center[1][0],
            w[0]*fv_data.obj_center[0][1] + w[1]*fv_data.obj_center[1][1] ]

  #obj_area0= [area for area in fv_data.obj_area_filtered]
  obj_area0_reg= [max(0.1,area) for area in fv_data.obj_area_filtered]

  #center_set= [get_center()]
  #Slip computation for log.
  get_slip1= lambda: sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])
  #Slip computation for control.
  #get_slip1_nml= lambda: sum(fv_data.mv_s[0])/obj_area0_reg[0]+sum(fv_data.mv_s[1])/obj_area0_reg[1]
  slip_detect2= lambda: ((sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>options['sensitivity_slip'],
                          np.max(fv_data.d_obj_center_filtered)>options['sensitivity_oc'],
                          np.max(fv_data.d_obj_orientation_filtered)>options['sensitivity_oo'],
                          np.max(fv_data.d_obj_area_filtered)>options['sensitivity_oa']))

  #Stop object detection
  ct.Run('fv.fv','stop_detect_obj',arm)

  l= TContainer(debug=True)
  l.x0= ct.robot.FK(arm=arm)
  l.ez0=  RotToExyz(QToRot(Transform(l.x0,ct.GetAttr('wrist_'+LRToStrs(arm),'lx'))[3:]))[0]
  l.x0[2]+= l.ez0[2]*ct.robot.FingertipOffset(arm=arm)
  l.tm0= rospy.Time.now()
  l.g_pos= ct.robot.GripperPos(arm)
  l.g_pos_log= [l.g_pos]
  #g_motion= None
  #slip_detected= False
  l.z_offset= 0.0
  #l.slip_amt= 0.0
  #l.slip_detected= False
  l.z_err= None
  l.z_gain_mode= 'low'
  l.g_motion= 0  #If >0, gripper is in motion (see below).
  l.suppress_z_ctrl= False  #If True, z_ctrl is suppressed during gripper control.
  l.suppress_slipavd= False  #If True, slip avoidance control is suppressed.
  l.log= options['log']
  l.log['area']= []
  l.log['slip']= []
  l.log['slip_nml']= []
  l.log['center']= []
  l.log['f']= []
  l.log['g_pos']= []
  l.log['z_trg']= []
  l.log['x']= []
  l.log['grasped']= False

  l.velctrl= ct.Run('velctrl',arm)
  l.ctrl_step= 0  #Counter for logging cycle control.

  l.force_detector= ct.Load('fv.grasp').TForceChangeDetector(fv_data)
  l.force_detector.Init()

  def InitObjArea():
    l.obj_area0= [area for area in fv_data.obj_area_filtered]
  InitObjArea()

  def IsDropped():
    #sum --> max for robustness.
    print 'area:',max(fv_data.obj_area_filtered),max(l.obj_area0),max(fv_data.obj_area_filtered)/max(max(l.obj_area0),1.0e-6)
    if max(fv_data.obj_area_filtered) < options['area_drop_rate']*max(l.obj_area0):
      l.log['grasped']= False
      return True
    return False

  def SetZOffset(z_offset):
    l.z_offset= z_offset
    l.z_err= None

  def SetZGainMode(z_gain_mode):
    l.z_gain_mode= z_gain_mode

  #Set z_offset=None to stop.
  def SetZCtrl(z_offset, z_gain_mode):
    if z_offset is not None:
      SetZOffset(z_offset)
    else:
      l.z_offset= 0.0
      _,z_err= ZTrgErr()
      l.z_offset= -z_err
    SetZGainMode(z_gain_mode)

  def ZTrgErr():
    x1= ct.robot.FK(arm=arm)
    l.z_trg= l.x0[2] + l.z_offset - l.ez0[2]*ct.robot.FingertipOffset(arm=arm)
    l.z_err= l.z_trg-x1[2]
    return l.z_trg, l.z_err

  #def ResetSlipDetect():
    #l.slip_amt= 0.0
    #l.slip_detected= False

  def GraspMore():
    count_wait_gmove= ct.GetAttr('fv_ctrl','pickup2a_gtimeout1')
    ##l.g_pos-= 0.0005 if arm==LEFT else 0.002
    #l.g_pos-= 0.0007 if arm==LEFT else 0.003
    l.g_pos-= ct.GetAttr('fv_ctrl','min_gstep')[arm]
    #ct.robot.MoveGripper(pos=l.g_pos, arm=arm, speed=100.0, blocking=False)
    #rospy.sleep(0.001)
    #l.g_pos= ct.robot.GripperPos(arm)
    ct.robot.MoveGripper(pos=l.g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=20.0, blocking=False)
    l.g_motion= count_wait_gmove

  def SetGrasp(g_pos, speed, exclusive):
    count_wait_gmove= ct.GetAttr('fv_ctrl','pickup2a_gtimeout2')
    l.g_pos= g_pos
    ct.robot.MoveGripper(pos=l.g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=speed, blocking=False)
    l.g_motion= count_wait_gmove
    l.suppress_z_ctrl= exclusive

  def LogGripperPos():
    l.g_pos_log.append(ct.robot.GripperPos(arm))

  def SetSuppressSlipAvd(suppress_slipavd):
    l.suppress_slipavd= suppress_slipavd

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
      l.log['area'].append([tm,sum(fv_data.obj_area)])
      l.log['slip'].append([tm,get_slip1()])
      l.log['center'].append([tm,get_center_avr()])
      l.log['f'].append([tm,map(float,fv_data.force[0][:3])+map(float,fv_data.force[1][:3])])
      l.log['g_pos'].append([tm,ct.robot.GripperPos(arm)])
      l.log['z_trg'].append([tm,float(ZTrgErr()[0])])
      l.log['x'].append([tm,ToList(ct.robot.FK(arm=arm))])

    l.force_detector.Update()

    #l.slip_curr= get_slip1()
    #if l.slip_curr>0.1:
    #l.slip_curr= get_slip1_nml()  #Normalized by object area.
    if any(slip_detect2()):
      CPrint(1,'slip_detection:', rospy.Time.now().to_sec(), slip_detect2())
      #print get_slip1(), get_slip1_nml()
      #l.slip_detected= True
      if not l.suppress_slipavd and l.g_motion==0 and not IsDropped():
        GraspMore()

    '''
    If l.g_motion>0, gripper is in motion.
    When l.g_motion>0, it denotes the timeout count.
    l.g_motion is set to zero when:
      1. The target gripper position l.g_pos is achieved.
      2. Timeout count (==l.g_motion) has passed.
    '''
    if l.g_motion>0:
      print rospy.Time.now().to_sec(), abs(ct.robot.GripperPos(arm)-l.g_pos), abs(ct.robot.GripperPos(arm)-l.g_pos)<0.0005
      #if abs(ct.robot.GripperPos(arm)-l.g_pos)<0.0005:  l.g_motion= 0
      if abs(ct.robot.GripperPos(arm)-l.g_pos)<0.5*ct.GetAttr('fv_ctrl','min_gstep')[arm]:  l.g_motion= 0
      else:  l.g_motion-= 1
      if l.g_motion==0:  l.suppress_z_ctrl= False

    if l.suppress_z_ctrl:
      dq= [0.0]*7
      l.velctrl.Step(dq)
    else:
      x1= ct.robot.FK(arm=arm)
      q= ct.robot.Q(arm=arm)
      J= ct.robot.J(q,arm=arm)
      vq1= MCVec(ct.robot.DQ(arm=arm))
      vx1= J * vq1

      #amp= 0.02 if z_offset<0.04 else 0.0
      #omega= 2.0
      kp= ct.GetAttr('fv_ctrl','pickup2a_kp')
      kd= ct.GetAttr('fv_ctrl','pickup2a_kd')
      ##kd= np.diag([0.3,0.5,0.5])
      ##vx= ToList(MCVec(vp) - kd*vx0[:3])+[0.0,0.0,0.0]
      #vx= [0.0,0.0, 0.005+0.02*math.cos(5.0*tm), 0.0,0.0,0.0]
      l.z_trg,l.z_err= ZTrgErr()
      #z_err_max= 0.006
      #z_err_max= 0.02
      z_err_max= 0.008
      if l.z_err>z_err_max:  z_err= z_err_max
      elif l.z_err<-z_err_max:  z_err= -z_err_max
      else:  z_err= l.z_err

      #When the slip is large, we move the robot slowly.
      #keypts= ([0.3,1.0], [0.7,0.0])
      #slip_to_kp_rate= max(keypts[1][1],min(keypts[1][0],(keypts[1][1]-keypts[0][1])/(keypts[1][0]-keypts[0][0])*(l.slip_curr-keypts[0][0])+keypts[0][1]))
      #s0,s1= 0.3, 0.7  #Slip range
      #r0,r1= 1.0, 0.5  #Corresponding kp rate
      #slip_to_kp_rate= max(r1,min(r0,(r1-r0)/(s1-s0)*(l.slip_curr-s0)+r0))
      #if z_err<0.0:  slip_to_kp_rate= 1.0
      slip_to_kp_rate= 1.0

      x_err= DiffX(x1,l.x0)
      vx= [kp[d]*x_err[d] - kd[d]*vx1[d,0] for d in range(6)]
      if l.z_gain_mode=='low':
        vx[2]= ct.GetAttr('fv_ctrl','pickup2a_lowgain')*slip_to_kp_rate*kp[2]*z_err-kd[2]*vx1[2,0]
      else:
        vx[2]= slip_to_kp_rate*kp[2]*z_err-kd[2]*vx1[2,0]
      #print vx

      if ct.robot.DoF(arm=arm)>=6:
        dq= ToList(la.pinv(J)*MCVec(vx))
      else:  #e.g. Mikata Arm
        W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
        dq= ToList(la.pinv(W*J)*W*MCVec(vx))
      l.velctrl.Step(dq)
    l.ctrl_step+= 1

  sm= TStateMachine(debug=True, local_obj=l)
  #sm.EventCallback= ct.SMCallback
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
  sm['bring_test'].EntryAction= lambda: SetZCtrl(0.02,'low')  #, ResetSlipDetect()
  sm['bring_test'].NewAction()
  sm['bring_test'].Actions[-1]= action_quit
  sm['bring_test'].NewAction()
  sm['bring_test'].Actions[-1].Condition= IsDropped
  #sm['bring_test'].Actions[-1].Action= lambda: (LogGripperPos(), SetGrasp(l.g_pos_log[0], speed=100.0, exclusive=True))
  sm['bring_test'].Actions[-1].NextState= 'grasp_init'
  sm['bring_test'].NewAction()
  sm['bring_test'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.002
  sm['bring_test'].Actions[-1].Action= lambda: l.log.__setitem__('grasped',True)
  sm['bring_test'].Actions[-1].NextState= 'bring_up'
  sm['bring_test'].ElseAction= action_ctrlstep

  sm.NewState('grasp_init')
  sm['grasp_init'].EntryAction= lambda: (LogGripperPos(), SetGrasp(l.g_pos_log[0], speed=100.0, exclusive=True))
  sm['grasp_init'].NewAction()
  sm['grasp_init'].Actions[-1]= action_quit
  sm['grasp_init'].NewAction()
  sm['grasp_init'].Actions[-1].Condition= lambda: l.g_motion==0
  sm['grasp_init'].Actions[-1].NextState= 'to_initial'
  sm['grasp_init'].ElseAction= action_ctrlstep

  sm.NewState('to_initial')
  sm['to_initial'].EntryAction= lambda: (SetZCtrl(0.0,'high'), SetSuppressSlipAvd(True), l.force_detector.Init())
  #ct.Run('fv.fv','start_detect_obj',arm)
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= action_quit
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= l.force_detector.IsDetected  #Stop when large force is detected.
  sm['to_initial'].Actions[-1].Action= lambda: (SetZCtrl(None,'high'), CPrint(4,'fv.pickup2b: Large force is detected. Stopping...'))
  #sm['to_initial'].Actions[-1].NextState= 'grasp_prev'
  sm['to_initial'].Actions[-1].NextState= action_quit.NextState
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.002
  #sm['to_initial'].Actions[-1].Action= lambda: SetGrasp(l.g_pos_log[-1], speed=10.0, exclusive=False)
  sm['to_initial'].Actions[-1].NextState= 'grasp_prev'
  sm['to_initial'].ElseAction= action_ctrlstep

  sm.NewState('grasp_prev')
  sm['grasp_prev'].EntryAction= lambda: SetGrasp(0.7*l.g_pos_log[-1]+0.3*l.g_pos_log[-2], speed=10.0, exclusive=False)
  sm['grasp_prev'].NewAction()
  sm['grasp_prev'].Actions[-1]= action_quit
  sm['grasp_prev'].NewAction()
  sm['grasp_prev'].Actions[-1].Condition= lambda: l.g_motion==0
  #sm['grasp_prev'].Actions[-1].Action= lambda: ct.Run('fv.fv','stop_detect_obj',arm)
  sm['grasp_prev'].Actions[-1].NextState= 'wait2'
  sm['grasp_prev'].ElseAction= action_ctrlstep

  sm.NewState('wait2')
  sm['wait2'].EntryAction= lambda: setattr(l,'tm2',rospy.Time.now()+rospy.Duration(0.1))
  sm['wait2'].NewAction()
  sm['wait2'].Actions[-1]= action_quit
  sm['wait2'].NewAction()
  sm['wait2'].Actions[-1].Condition= lambda: (l.tm2<rospy.Time.now(), CPrint(0,l.tm2.to_sec(),rospy.Time.now().to_sec()))[0]
  sm['wait2'].Actions[-1].Action= lambda: (InitObjArea(), SetSuppressSlipAvd(False))
  sm['wait2'].Actions[-1].NextState= 'bring_test'
  sm['wait2'].ElseAction= action_ctrlstep

  #sm.NewState('grasp_more')
  #sm['grasp_more'].EntryAction= GraspMore
  #sm['grasp_more'].NewAction()
  #sm['grasp_more'].Actions[-1]= action_quit
  #sm['grasp_more'].NewAction()
  #sm['grasp_more'].Actions[-1].Condition= lambda: l.g_motion==0
  #sm['grasp_more'].Actions[-1].NextState= 'bring_test'
  #sm['grasp_more'].ElseAction= action_ctrlstep

  #sm.NewState('wait')
  #sm['wait'].EntryAction= lambda: setattr(l,'tm1',rospy.Time.now()+rospy.Duration(0.4))
  #sm['wait'].NewAction()
  #sm['wait'].Actions[-1]= action_quit
  #sm['wait'].NewAction()
  #sm['wait'].Actions[-1].Condition= IsDropped
  #sm['wait'].Actions[-1].Action= lambda: (LogGripperPos(), SetGrasp(l.g_pos_log[0], speed=100.0, exclusive=True))
  #sm['wait'].Actions[-1].NextState= 'to_initial'
  #sm['wait'].NewAction()
  #sm['wait'].Actions[-1].Condition= lambda: l.tm1<rospy.Time.now()
  #sm['wait'].Actions[-1].Action= GraspMore  #NEW
  #sm['wait'].Actions[-1].NextState= 'bring_up'
  #sm['wait'].ElseAction= action_ctrlstep

  sm.NewState('bring_up')
  sm['bring_up'].EntryAction= lambda: SetZCtrl(options['z_final'],'high')
  sm['bring_up'].NewAction()
  sm['bring_up'].Actions[-1]= action_quit
  sm['bring_up'].NewAction()
  sm['bring_up'].Actions[-1].Condition= IsDropped
  sm['bring_up'].Actions[-1].Action= lambda: (LogGripperPos(), SetGrasp(l.g_pos_log[0], speed=100.0, exclusive=True))
  sm['bring_up'].Actions[-1].NextState= 'to_initial'
  if options['auto_stop']:
    sm['bring_up'].NewAction()
    sm['bring_up'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.005
    sm['bring_up'].Actions[-1].NextState= EXIT_STATE
  sm['bring_up'].ElseAction= action_ctrlstep

  sm.NewState('bring_up_to_exit')
  sm['bring_up_to_exit'].EntryAction= lambda: SetZCtrl(options['z_final'],'high')
  sm['bring_up_to_exit'].NewAction()
  sm['bring_up_to_exit'].Actions[-1].Condition= lambda: abs(ZTrgErr()[1])<0.005
  sm['bring_up_to_exit'].Actions[-1].NextState= EXIT_STATE
  sm['bring_up_to_exit'].ElseAction= action_ctrlstep

  try:
    sm.Run()

  finally:
    ct.callback.fv_objinfo[ct.robot.ArmStrS(arm)]= [None,None]
    l.velctrl.Finish()
    if options['resume_detect_obj']: ct.Run('fv.fv','start_detect_obj',arm)

  if options['keep_thread_after_exit']:
    while not action_quit.Condition():
      rospy.sleep(0.01)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    user_options= args[1] if len(args)>1 else {}

    options= PickupLoopDefaultOptions(ct)
    InsertDict(options, user_options)
    if 'log' in user_options:  options['log']= user_options['log']

    if 'vs_pickup2b'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_pickup2b'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    CPrint(1,'Turn on:','vs_pickup2b'+LRToStrS(arm))
    ct.thread_manager.Add(name='vs_pickup2b'+LRToStrS(arm), target=lambda th_info: PickupLoop(th_info,ct,arm,options))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    CPrint(2,'Turn off:','vs_pickup2b'+LRToStrS(arm))
    ct.thread_manager.Stop(name='vs_pickup2b'+LRToStrS(arm))

  elif command=='clear':
    CPrint(2,'Turn off:','vs_pickup2b'+LRToStrS(RIGHT))
    CPrint(2,'Turn off:','vs_pickup2b'+LRToStrS(LEFT))
    ct.thread_manager.Stop(name='vs_pickup2b'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_pickup2b'+LRToStrS(LEFT))

  elif command=='once':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    user_options= args[1] if len(args)>1 else {}

    options= PickupLoopDefaultOptions(ct)
    InsertDict(options, user_options)
    if 'log' in user_options:  options['log']= user_options['log']
    options['auto_stop']= True

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    PickupLoop(None, ct, arm, options=options)


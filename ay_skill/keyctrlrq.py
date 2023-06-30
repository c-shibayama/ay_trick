#!/usr/bin/python
from core_tool import *
SmartImportReload('keyctrl3')
from keyctrl3 import Callback
import sensor_msgs.msg

def Help():
  return '''Joystick controller for standalone grippers.
  Usage: keyctrlrq'''

def Run(ct,*args):
  if not any((ct.robot.Is('RobotiqNB'),ct.robot.Is('DxlGripper'),ct.robot.Is('RHP12RNGripper'),ct.robot.Is('EZGripper'),ct.robot.Is('DxlpO2Gripper'),ct.robot.Is('DxlO3Gripper'))):
    CPrint(4,'This program works only with RobotiqNB, DxlGripper, RHP12RNGripper, EZGripper, DxlpO2Gripper, and DxlO3Gripper.')
    return

  arm= ct.robot.Arm
  #ct.robot.grippers[arm].Init()  #FIXME: Commented out as I'm not sure why this code is here.

  if ct.robot.Is('DxlGripper'):
    active_holding= [False]

  steps= [0.0, 0.0, 0.0]
  wsteps= [0.0, 0.0, 0.0]
  gsteps= [0.0]
  state= ['run', 'no_cmd', arm, False]  #run/quit, no_cmd/CMD, ARM, ACTIVE_BTN

  gstate_range= [ct.robot.GripperRange(a) for a in range(ct.robot.NumArms)]
  gstate= [ct.robot.GripperPos(a) if ct.robot.EndEff(a).IsInitialized() else 0.0 for a in range(ct.robot.NumArms)]
  for a in range(ct.robot.NumArms):
    if ct.robot.EndEff(a).IsInitialized():
      ct.robot.MoveGripper(gstate[a],arm=a)

  ct.AddSub('joy', 'joy', sensor_msgs.msg.Joy, lambda msg: Callback(state, steps, wsteps, gsteps, msg))

  kbhit= TKBHit()
  try:
    while state[0]=='run':
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':
          break;
        elif key is not None:
          state[1]= 'key_'+str(key)
      else:
        break

      if state[1]=='key_h':
        print '''\
Command:
  Joystick:
    RB: Activation
    LB: Mode switch (linear/rotational)
    RT: Decelerator (analog; 1 to 0)
    LT: Accelerator (analog; 1 to 5)
    Start: Quit
    Left-stick (axes 0,1):
      Left/Right: +y/-y (linear mode), -wx/+wx (rotational mode)
      Up/Down:    +x/-x (linear mode), +wy/-wy (rotational mode)
    Right-stick (axes 0,1):
      Left/Right: +wz/-wz
      Up/Down:    +z/-z
    X: Switch arm to LEFT
    B: Switch arm to RIGHT
    Y: ==key_i
    A: Gripper mode when keep pressing
      Left-stick (axes 0,1):
        Left/Right: open/close
    DPad:
      Left:  Run fv.open arm  (Cancel all tactile-based grasping and Open)
      Right: Run fv.grasp 'on'/'off' arm  (Gentle grasp)
      Up:    Run fv.hold' on'/'off' arm   (Holding = Slip avoidance)
      Down:  Run fv.openif 'on'/'off' arm (Handover)
  Keyboard:
    q: Quit
    h: Show help
    l/r: Switch arm to LEFT/RIGHT
    i: Run fv.inhand 'on'/'off' arm  (In-hand manipulation) '''
        state[1]= 'no_cmd'

      elif state[1]=='key_l' or state[1]=='key_r':
        arm= LEFT if state[1]=='key_l' else RIGHT
        print 'Switched arm:',LRToStr(arm)
        state[1]= 'no_cmd'
      elif state[1]=='key_i' or state[1]=='cmd_Y':
        if 'vs_inhand'+LRToStrS(arm) not in ct.thread_manager.thread_list:
          ct.Run('fv.inhand','on',arm)
        else:
          ct.Run('fv.inhand','off',arm)
        state[1]= 'no_cmd'

      elif state[1]=='arm_switch':
        arm= RIGHT
        #ct.robot.limbs[arm].exit_control_mode()
        #arm= state[2]
        print 'Switched arm:',LRToStr(arm)
        state[1]= 'no_cmd'

      #elif state[1]=='cmd_Y':
        #ct.Run('calib_x')
        #state[1]= 'no_cmd'

      elif state[1]=='cmd_left':
        ct.Run('fv.open',arm)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_right':
        if state[3]:
          ct.Run('fv.grasp','on',arm)
        else:
          ct.Run('fv.grasp','off',arm)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_up':
        if state[3]:
          ct.Run('fv.hold','on',arm)
        else:
          ct.Run('fv.hold','off',arm)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_down':
        #ct.Run('fv.cut3',arm)
        if state[3]:
          ct.Run('fv.openif','on',arm)
        else:
          ct.Run('fv.openif','off',arm)
        state[1]= 'no_cmd'

      elif state[1]=='key_[':
        gsteps[0]= 0.1
        state[1]= 'grip'
      elif state[1]=='key_]':
        gsteps[0]= -0.1
        state[1]= 'grip'
      elif state[1]=='key_{':
        gsteps[0]= 1.0
        state[1]= 'grip'
      elif state[1]=='key_}':
        gsteps[0]= -1.0
        state[1]= 'grip'

      #elif state[1]=='grip':
      if state[1]=='grip':
        if ct.robot.Is('DxlGripper'):
          if not active_holding[arm]:
            ct.robot.EndEff(arm).StartHolding()
            active_holding[arm]= True
        #gstate[arm]+= 0.0002*gsteps[0]
        #gstate[arm]= ct.robot.GripperPos(arm) + 0.005*gsteps[0]
        gstate[arm]= ct.robot.GripperPos(arm) + 0.01*gsteps[0]
        #print gstate[arm]
        if gstate[arm]<gstate_range[arm][0]:  gstate[arm]= gstate_range[arm][0]
        if gstate[arm]>gstate_range[arm][1]:  gstate[arm]= gstate_range[arm][1]
        #print rospy.Time.now(),LRToStr(arm),ct.robot.GripperPos(arm),gstate[arm]
        ct.robot.MoveGripper(gstate[arm],max_effort=100.0,speed=100.0,arm=arm)
        #gsteps[0]= 0
        #state[1]= 'no_cmd'
        #rospy.sleep(0.015)

      if not state[1]=='grip':
        if ct.robot.Is('DxlGripper'):
          if active_holding[arm]:
            ct.robot.EndEff(arm).StopHolding()
            active_holding[arm]= False

      rospy.sleep(0.005)

      '''
      #elif state[1]=='grip':
      #if state[1]=='grip':
      #gstate[arm]+= 0.0002*gsteps[0]
      gstate[arm]= ct.robot.GripperPos(arm) + 0.005*gsteps[0]
      #print gstate[arm]
      if gstate[arm]<gstate_range[arm][0]:  gstate[arm]= gstate_range[arm][0]
      if gstate[arm]>gstate_range[arm][1]:  gstate[arm]= gstate_range[arm][1]
      #print rospy.Time.now(),LRToStr(arm),ct.robot.GripperPos(arm),gstate[arm]
      ct.robot.MoveGripper(gstate[arm],max_effort=100.0,speed=100.0,arm=arm)
      gsteps[0]= 0
      state[1]= 'no_cmd'
      rospy.sleep(0.015)
      '''

  finally:
    kbhit.Deactivate()
    ct.DelSub('joy')
    if ct.robot.Is('DxlGripper'):
      if active_holding[arm]:
        ct.robot.EndEff(arm).StopHolding()
        active_holding[arm]= False
    print 'Finished'

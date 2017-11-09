#!/usr/bin/python
from core_tool import *
SmartImportReload('lib.keyctrl3')
from lib.keyctrl3 import Callback
import sensor_msgs.msg

def Help():
  return '''Joystick controller for RobotiqNB and DxlGripper.
  Usage: keyctrlrq'''

def Run(ct,*args):
  if not (ct.robot.Is('RobotiqNB') or ct.robot.Is('DxlGripper')):
    CPrint(4,'This program works only with RobotiqNB and DxlGripper.')
    return

  arm= ct.robot.Arm
  ct.robot.grippers[arm].Init()

  steps= [0.0, 0.0, 0.0]
  wsteps= [0.0, 0.0, 0.0]
  gsteps= [0.0]
  state= ['run', 'no_cmd', arm, False]  #run/quit, no_cmd/CMD, ARM, ACTIVE_BTN

  gstate_range= [ct.robot.GripperRange(RIGHT),ct.robot.GripperRange(LEFT)]
  gstate= [ct.robot.GripperPos(RIGHT), ct.robot.GripperPos(LEFT)]
  ct.robot.MoveGripper(gstate[RIGHT],arm=RIGHT)
  ct.robot.MoveGripper(gstate[LEFT],arm=LEFT)

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
      Left:  Run fv.vs_open arm  (Cancel all tactile-based grasping and Open)
      Right: Run fv.vs_grasp 'on'/'off' arm  (Gentle grasp)
      Up:    Run fv.vs_hold' on'/'off' arm   (Holding = Slip avoidance)
      Down:  Run fv.vs_openif 'on'/'off' arm (Handover)
  Keyboard:
    q: Quit
    h: Show help
    l/r: Switch arm to LEFT/RIGHT
    i: Run fv.vs_inhand 'on'/'off' arm  (In-hand manipulation) '''
        state[1]= 'no_cmd'

      elif state[1]=='key_l' or state[1]=='key_r':
        arm= LEFT if state[1]=='key_l' else RIGHT
        print 'Switched arm:',LRToStr(arm)
        state[1]= 'no_cmd'
      elif state[1]=='key_i' or state[1]=='cmd_Y':
        if 'vs_inhand'+LRToStrS(arm) not in ct.thread_manager.thread_list:
          ct.Run('fv.vs_inhand','on',arm)
        else:
          ct.Run('fv.vs_inhand','off',arm)
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
        ct.Run('fv.vs_open',arm)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_right':
        if state[3]:
          ct.Run('fv.vs_grasp','on',arm)
        else:
          ct.Run('fv.vs_grasp','off',arm)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_up':
        if state[3]:
          ct.Run('fv.vs_hold','on',arm)
        else:
          ct.Run('fv.vs_hold','off',arm)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_down':
        #ct.Run('fv.cut3',arm)
        if state[3]:
          ct.Run('fv.vs_openif','on',arm)
        else:
          ct.Run('fv.vs_openif','off',arm)
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
    print 'Finished'

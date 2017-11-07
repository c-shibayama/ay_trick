#!/usr/bin/python
from core_tool import *
def Help():
  return '''Velocity control using Joystick and Jacobian test with Baxter.
  Usage: j'''

def Callback(state, steps, wsteps, gsteps, data):
  gsteps[0]= 0.0
  steps[:]= [0.0]*3
  wsteps[:]= [0.0]*3
  state[1]= 'no_cmd'
  state[3]= False

  #multiplier= 1.0
  multiplier= (1.0+data.axes[5])*0.5 + (1.0-data.axes[2])*2.0  #RT, LT
  #if data.buttons[1] > 0:
    #multiplier= 2.0 * multiplier

  axes= [(ax if abs(ax)>0.15 else 0.0) for ax in data.axes]

  if data.buttons[7] > 0:  #START
    state[0]= 'quit'
    return

  if data.buttons[1] > 0:  #B
    state[1]= 'arm_switch'
    state[2]= RIGHT
    #return
  if data.buttons[2] > 0:  #X
    state[1]= 'arm_switch'
    state[2]= LEFT
    #return

  if data.buttons[3] > 0:  #Y
    state[1]= 'cmd_Y'
    #return

  dpad_btn= [1 if btn>0 else 0 for btn in data.buttons[11:15]]
  if any(dpad_btn):
    if sum(dpad_btn)==1:  #Need to press exactly one d-pad button at the same time
      if data.buttons[11] > 0:  #LEFT
        state[1]= 'cmd_left'
        #return
      if data.buttons[12] > 0:  #RIGHT
        state[1]= 'cmd_right'
        #return
      if data.buttons[13] > 0:  #UP
        state[1]= 'cmd_up'
        #return
      if data.buttons[14] > 0:  #DOWN
        state[1]= 'cmd_down'
        #return

  if data.buttons[0] > 0:  #A
    state[1]= 'grip'
    gsteps[0]= multiplier * axes[0]
    #return

  if data.buttons[5] <= 0:  #not RB
    state[3]= False
  else:
    state[3]= True

  if state[3] and state[1]=='no_cmd':
    if data.buttons[4] <= 0:  #not LB
      state[1]= 'position'
      gsteps[0]= 0.0
      steps[0]= multiplier * axes[1]
      steps[1]= multiplier * axes[0]
      steps[2]= multiplier * axes[4]
      wsteps[0]= 0.0
      wsteps[1]= 0.0
      wsteps[2]= multiplier * axes[3]
    else:  #LB
      state[1]= 'orientation'
      gsteps[0]= 0.0
      steps[0]= 0.0
      steps[1]= 0.0
      steps[2]= multiplier * axes[4]
      wsteps[0]= -multiplier * axes[0]
      wsteps[1]= multiplier * axes[1]
      wsteps[2]= multiplier * axes[3]

def Run(ct,*args):
  if not ct.robot.Is('Baxter'):
    CPrint(4,'This program works only with Baxter.')
    return

  arm= ct.robot.Arm

  steps= [0.0, 0.0, 0.0]
  wsteps= [0.0, 0.0, 0.0]
  gsteps= [0.0]
  state= ['run', 'no_cmd', arm, False]  #run/quit, no_cmd/CMD, ARM, ACTIVE_BTN

  gstate_range= [ct.robot.GripperRange(RIGHT),ct.robot.GripperRange(LEFT)]
  gstate= [ct.robot.GripperPos(RIGHT), ct.robot.GripperPos(LEFT)]
  ct.robot.MoveGripper(gstate[RIGHT],arm=RIGHT)
  ct.robot.MoveGripper(gstate[LEFT],arm=LEFT)

  ct.AddSub('joy', 'joy', sensor_msgs.msg.Joy, lambda msg: Callback(state, steps, wsteps, gsteps, msg))
  velctrl= [ct.Load('bx_velctrl').TVelCtrl(ct,arm=RIGHT), ct.Load('bx_velctrl').TVelCtrl(ct,arm=LEFT)]
  suppress_velctrl= False  #Set this True when an external program use the velocity control.

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
    Y: Run calib_x
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
    c: Run calib_x (Calibrate the external RGB-D sensor pose)
    i: Run fv.vs_inhand 'on'/'off' arm  (In-hand manipulation)
    ct: Run fv.vs_trackf4 'on'/'off' arm (Tai Chi)
    y: Run fv.vs_trackf4 for BOTH ARM   (Tai Chi)
    o: Run fv.vs_tracko 'on'/'off' arm  (Proximity vision-based tracking)
    p: Run fv.vs_pickup2a 'on'/'off' arm (Slip-based automatic picking up) '''
        state[1]= 'no_cmd'

      elif state[1]=='key_l' or state[1]=='key_r':
        arm= LEFT if state[1]=='key_l' else RIGHT
        print 'Switched arm:',LRToStr(arm)
        state[1]= 'no_cmd'
      elif state[1]=='key_c':
        ct.Run('calib_x')
        state[1]= 'no_cmd'
      elif state[1]=='key_i':
        if 'vs_inhand'+LRToStrS(arm) not in ct.thread_manager.thread_list:
          ct.Run('fv.vs_inhand','on',arm)
        else:
          ct.Run('fv.vs_inhand','off',arm)
        state[1]= 'no_cmd'
      elif state[1]=='key_t':
        if 'vs_trackf4'+LRToStrS(arm) not in ct.thread_manager.thread_list:
          ct.Run('fv.vs_trackf4','on',arm)
          suppress_velctrl= True
        else:
          ct.Run('fv.vs_trackf4','off',arm)
          suppress_velctrl= False
        state[1]= 'no_cmd'
      elif state[1]=='key_y':
        if 'vs_trackf4'+LRToStrS(LEFT) not in ct.thread_manager.thread_list and 'vs_trackf4'+LRToStrS(RIGHT) not in ct.thread_manager.thread_list:
          ct.Run('fv.vs_trackf4','on',LEFT)
          ct.Run('fv.vs_trackf4','on',RIGHT)
          suppress_velctrl= True
        else:
          ct.Run('fv.vs_trackf4','clear')
          suppress_velctrl= False
        state[1]= 'no_cmd'
      elif state[1]=='key_o':
        if 'vs_tracko'+LRToStrS(arm) not in ct.thread_manager.thread_list:
          ct.Run('fv.vs_tracko','on',arm)
          suppress_velctrl= True
        else:
          ct.Run('fv.vs_tracko','off',arm)
          suppress_velctrl= False
        state[1]= 'no_cmd'
      elif state[1]=='key_p' or state[1]=='cmd_Y':
        if 'vs_pickup2a'+LRToStrS(arm) not in ct.thread_manager.thread_list:
          ct.Run('fv.vs_pickup2a','on',arm)  #,{'resume_detect_obj':False}
          suppress_velctrl= True
        else:
          ct.Run('fv.vs_pickup2a','off',arm)
          suppress_velctrl= False
        state[1]= 'no_cmd'

      elif state[1]=='arm_switch':
        ct.robot.limbs[arm].exit_control_mode()
        arm= state[2]
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

      elif state[1]=='grip':
        #gstate[arm]+= 0.0002*gsteps[0]
        gstate[arm]= ct.robot.GripperPos(arm) + 0.005*gsteps[0]
        if gstate[arm]<gstate_range[arm][0]:  gstate[arm]= gstate_range[arm][0]
        if gstate[arm]>gstate_range[arm][1]:  gstate[arm]= gstate_range[arm][1]
        #print rospy.Time.now(),LRToStr(arm),ct.robot.GripperPos(arm),gstate[arm]
        ct.robot.MoveGripper(gstate[arm],max_effort=100.0,speed=100.0,arm=arm)
        if ct.robot.Is('Baxter') and isinstance(ct.robot.grippers[arm], baxter_interface.Gripper):
          rospy.sleep(0.01)  #Without this, ct.robot.GripperPos(arm) is not updated.

      if state[3] and state[1] in ('position','orientation'):
        q= ct.robot.Q(arm=arm)
        f= 1.0
        #f= 0.2
        vx= map(lambda x:f*0.4*x,steps)+map(lambda x:f*1.0*x,wsteps)
        dq= ToList(la.pinv(ct.robot.J(q,arm=arm))*MCVec(vx))
      else:
        dq= [0.0]*ct.robot.DoF(arm)
      if not suppress_velctrl:
        velctrl[arm].Step(dq)

  finally:
    velctrl[RIGHT].Finish()
    velctrl[LEFT].Finish()
    kbhit.Deactivate()
    ct.DelSub('joy')
    print 'Finished'

#!/usr/bin/python
from core_tool import *
roslib.load_manifest('sensor_msgs')
import sensor_msgs.msg

def Help():
  return '''RealSense2 camera pose calibration tool with joystick operation for wrist_rs2 and robot_rs2.
  Usage:  rs2_joycalib [RS_ATTR [, CONSTRAINT]]
    RS_ATTR: Attribute name to access the data; default:'rs' (ct.GetAttr(TMP,RS_ATTR)).
    CONSTRAINT: Type of pose-displacement constraint (default: 'xyz').
      'none':    No constraint.
      'xy':      XY movement only.
      'z':       Z movement only.
      'xyz':     XYZ movement only.
      'yaw':     Yaw rotation only.
      'rpy':     RPY rotation only.
      'xy_yaw':  XY movement and Yaw rotation only.
      'xyz_yaw': XYZ movement and Yaw rotation only.
  '''

def CallbackJoy(data, joyst):
  joyst['LT']= data.axes[2]
  joyst['RT']= data.axes[5]

  #Insensitive zone of sticks
  axes= [(ax if abs(ax)>0.15 else 0.0) for ax in data.axes]

  joyst['START']= (data.buttons[7] > 0)
  #joyst['A']= (data.buttons[0] > 0)
  #joyst['B']= (data.buttons[1] > 0)
  #joyst['X']= (data.buttons[2] > 0)
  joyst['Y']= (data.buttons[3] > 0)
  joyst['LB']= (data.buttons[4] > 0)
  joyst['RB']= (data.buttons[5] > 0)

  ##D-pad
  #dpad_btn= [1 if btn>0 else 0 for btn in data.buttons[11:15]]
  #joyst['LEFT']= False
  #joyst['RIGHT']= False
  #joyst['UP']= False
  #joyst['DOWN']= False
  #if any(dpad_btn):
    #if sum(dpad_btn)==1:  #Need to press exactly one d-pad button at the same time
      #joyst['LEFT']= (data.buttons[11] > 0)
      #joyst['RIGHT']= (data.buttons[12] > 0)
      #joyst['UP']= (data.buttons[13] > 0)
      #joyst['DOWN']= (data.buttons[14] > 0)

  #Stick
  joyst['AXIS0']= axes[0]  #Stick(left): left(+)-right(-)
  joyst['AXIS1']= axes[1]  #Stick(left): top(+)-down(-)
  joyst['AXIS3']= axes[3]  #Stick(right): left(+)-right(-)
  joyst['AXIS4']= axes[4]  #Stick(right): top(+)-down(-)

def Run(ct,*args):
  rs_attr= args[0] if len(args)>0 else 'rs'
  constraint= args[1] if len(args)>1 else 'xyz'

  speed_gain= 0.04
  rate= 50.

  dt= 1./rate
  dx_cam= ct.Run('tf_once','camera_link','camera_color_optical_frame')
  l= ct.GetAttr(TMP,rs_attr)

  joyst= {}
  ct.AddSub('joy', 'joy', sensor_msgs.msg.Joy, lambda msg,joyst=joyst: CallbackJoy(msg, joyst))
  print 'Constraint mode:',constraint
  print '  Original lx=',l.options['lx']
  print '  Current lx=',l.lx
  print '  Press Y to reset to the original.'

  try:
    kbhit= TKBHit()
    rate_adjuster= rospy.Rate(rate)

    while not rospy.is_shutdown():
      key= None
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':  break;
      else:  break
      if joyst and joyst['START']:  break
      if (joyst and joyst['Y']) or key=='r':
        print 'Reset the calibration.'
        l.lx= l.options['lx']
        l.lw_x_camera_link= TransformRightInv(l.lx,dx_cam)

      #Change x_trg and g_trg according to joystick reading.
      is_changed= False
      steps= [0.0,0.0,0.0]
      wsteps= [0.0,0.0,0.0]
      if joyst:
        multiplier= (1.0+joyst['RT'])*0.5 + (1.0-joyst['LT'])*2.0  #RT, LT
        if joyst['RB']:
          if not joyst['LB']:  #Linear mode.
            steps[0]= multiplier * joyst['AXIS1']
            steps[1]= multiplier * joyst['AXIS0']
            steps[2]= multiplier * joyst['AXIS4']
            wsteps[0]= 0.0
            wsteps[1]= 0.0
            wsteps[2]= multiplier * joyst['AXIS3']
          else:  #Rotation mode.
            steps[0]= 0.0
            steps[1]= 0.0
            steps[2]= multiplier * joyst['AXIS4']
            wsteps[0]= -multiplier * joyst['AXIS0']
            wsteps[1]= multiplier * joyst['AXIS1']
            wsteps[2]= multiplier * joyst['AXIS3']
          if constraint=='none':
            pass
          elif constraint=='xy':
            steps[2]= 0.0
            wsteps[:3]= [0.0,0.0,0.0]
          elif constraint=='z':
            steps[:2]= [0.0,0.0]
            wsteps[:3]= [0.0,0.0,0.0]
          elif constraint=='xyz':
            wsteps[:3]= [0.0,0.0,0.0]
          elif constraint=='yaw':
            steps[:3]= [0.0,0.0,0.0]
            wsteps[:2]= [0.0,0.0]
          elif constraint=='rpy':
            steps[:3]= [0.0,0.0,0.0]
          elif constraint=='xy_yaw':
            steps[2]= 0.0
            wsteps[:2]= [0.0,0.0]
          elif constraint=='xyz_yaw':
            wsteps[:2]= [0.0,0.0]
          elif constraint=='gripper':
            steps[:3]= [0.0,0.0,0.0]
            wsteps[:3]= [0.0,0.0,0.0]
          else:
            raise Exception('joycalib: Unexpected constraint type: {constraint}'.format(constraint=constraint))
          is_changed= any(np.abs(steps)>0) or any(np.abs(wsteps)>0)

      if is_changed:
        vx_joy= map(lambda x:speed_gain*x,steps)+map(lambda x:speed_gain*5.0*x,wsteps)
        x_diff= np.array(vx_joy)*dt
        l.lx= AddDiffX(l.lx, x_diff)
        l.lw_x_camera_link= TransformRightInv(l.lx,dx_cam)

      rate_adjuster.sleep()

    print ''
    print 'Calibration done.'
    print 'Original lx=',l.options['lx']
    print 'lx=',l.lx

  finally:
    kbhit.Deactivate()
    ct.DelSub('joy')


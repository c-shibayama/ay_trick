#!/usr/bin/python
from core_tool import *
import sensor_msgs.msg
def Help():
  return '''Joystick controller for 2F2 grippers.
  Usage: test.jg2f2'''

def CallbackJoy(data, joyst):
  joyst['LT']= data.axes[2]
  joyst['RT']= data.axes[5]

  #Insensitive zone of sticks
  axes= [(ax if abs(ax)>0.15 else 0.0) for ax in data.axes]

  joyst['START']= (data.buttons[7] > 0)
  joyst['A']= (data.buttons[0] > 0)
  joyst['B']= (data.buttons[1] > 0)
  joyst['X']= (data.buttons[2] > 0)
  joyst['Y']= (data.buttons[3] > 0)
  joyst['LB']= (data.buttons[4] > 0)
  joyst['RB']= (data.buttons[5] > 0)

  #D-pad
  dpad_btn= [1 if btn>0 else 0 for btn in data.buttons[11:15]]
  joyst['LEFT']= False
  joyst['RIGHT']= False
  joyst['UP']= False
  joyst['DOWN']= False
  if any(dpad_btn):
    if sum(dpad_btn)==1:  #Need to press exactly one d-pad button at the same time
      joyst['LEFT']= (data.buttons[11] > 0)
      joyst['RIGHT']= (data.buttons[12] > 0)
      joyst['UP']= (data.buttons[13] > 0)
      joyst['DOWN']= (data.buttons[14] > 0)

  #Stick
  joyst['AXIS0']= axes[0]  #Stick(left): left(+)-right(-)
  joyst['AXIS1']= axes[1]  #Stick(left): top(+)-down(-)
  joyst['AXIS3']= axes[3]  #Stick(right): left(+)-right(-)
  joyst['AXIS4']= axes[4]  #Stick(right): top(+)-down(-)


def Run(ct,*args):
  arm= ct.robot.Arm
  joyst= {}
  ct.AddSubW('joy', 'joy', sensor_msgs.msg.Joy, lambda msg,joyst=joyst: CallbackJoy(msg, joyst))

  try:
    kbhit= TKBHit()
    dt= 0.01
    rate_adjuster= rospy.Rate(1./dt)

    g_trg= ct.robot.GripperPos2(arm)

    while True:
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':  break;
      else:  break
      if 'START' in joyst and joyst['START']:  break

      #Change x_trg and g_trg according to joystick reading.
      gripper_motion= False
      gsteps= [0.0,0.0]
      multiplier= (1.0+joyst['RT'])*0.5 + (1.0-joyst['LT'])*2.0  #RT, LT
      if joyst['LB']:  #Coupling mode.
        gsteps[0]= multiplier * (joyst['AXIS0'] + joyst['AXIS3'])
        gsteps[1]= multiplier * (joyst['AXIS0'] - joyst['AXIS3'])
      elif joyst['RB']:  #Independent mode.
        gsteps[0]= multiplier * joyst['AXIS0']
        gsteps[1]= multiplier * (-joyst['AXIS3'])
      gripper_motion= any(np.abs(gsteps)>0)

      if gripper_motion:
        gripper_range= ct.robot.GripperRange2(arm=arm)
        for d in (0,1):
          g_trg[d]+= 0.005*gsteps[d]*dt
          if not IsIn(g_trg[d],[gripper_range[0][d],gripper_range[1][d]]):
            g_trg[d]= min(max(g_trg[d],gripper_range[0][d]),gripper_range[1][d])
        ct.robot.MoveGripper2(g_trg,arm=arm)
        print g_trg

    print ''

  finally:
    kbhit.Deactivate()
    ct.DelSub('joy')

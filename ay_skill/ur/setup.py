#!/usr/bin/python
from core_tool import *

def Help():
  return '''Setup for UR+FV demo.
  Assumption: Following ROS nodes are launched beforehand.
    - UR robot driver
    - FV
    Use $ rosrun ay_util ur_gui.py
  Usage: ur.setup ROBOT_NAME [, NO_ASK]
    ROBOT_NAME: Robot mode ('UR3eThG','UR3eThG_SIM', etc., default='UR3eThG').
    NO_ASK: If True, the system does not ask if run each command (default=False).
  '''

def Run(ct,*args):
  robot_name= args[0] if len(args)>0 else 'UR3eThG'
  no_ask= args[1] if len(args)>1 else False
  #fv_names,node_names= {'A':{RIGHT:'fvp_1_r',LEFT:'fvp_1_l'}},{'A':'fvp_1'}
  if not robot_name.endswith('_SIM'):
    command_list=[
      ['robot', robot_name],
      ['fv.fv', 'on', 'all', {'A':{RIGHT:'fvp_1_r',LEFT:'fvp_1_l'}}],
      ['viz',''],
      ]
  else:
    command_list=[
      ['robot', robot_name],
      ['viz',''],
      ]
  for cmd in command_list:
    print ''
    CPrint(2,'Running command: > ',cmd[0],', '.join(map(repr,cmd[1:])) )
    if not no_ask:
      print '  y:Continue, s:Skip, q:Quit'
      res= KBHAskGen('y','s','q')
    else:
      res= 'y'
    if res=='y':
      ct.Run(*cmd)
    elif res=='s':
      pass
    elif res=='q':
      break
    #Bruteforce bug fix to ensure the robot is completely setup before observe.
    rate= rospy.Rate(20)
    t_start= rospy.Time.now()
    while (rospy.Time.now()-t_start).to_sec()<3.0 and not rospy.is_shutdown():
      try:
        q= ct.robot.Q()
        g= ct.robot.GripperPos()
        print 'The robot is ready!'
        break
      except Exception:
        print 'The robot is not ready...'
      rate.sleep()

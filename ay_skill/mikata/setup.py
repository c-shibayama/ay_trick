#!/usr/bin/python
from core_tool import *

def Help():
  return '''Setup for Mikata+FV demo.
  Assumption: Following ROS nodes are launched beforehand.
    - Mikata robot driver
    - FV
    Use $ rosrun ay_util mikata_gui.py
  Usage: mikata.setup [MODE [, NO_ASK]]
    MODE: Setup mode ('mikata','sim', default='mikata').
    NO_ASK: If True, the system does not ask if run each command (default=False).
  '''

def Run(ct,*args):
  mode= args[0] if len(args)>0 else 'mikata'
  no_ask= args[1] if len(args)>1 else False
  fv_names,node_names= {'A':{RIGHT:'fvp_mikata_r',LEFT:'fvp_mikata_l'}},{'A':'fvp_mikata'}
  if mode=='mikata':
    command_list=[
      ['robot', 'mikata2'],
      ['fv.fv', 'on', 'all', fv_names, node_names],
      ['mikata.effort', 100],
      ['viz',''],
      ]
  elif mode=='sim':
    command_list=[
      ['robot', 'mikatas'],
      ['viz',''],
      ]
  else:  raise Exception('Invalid mode:',mode)
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

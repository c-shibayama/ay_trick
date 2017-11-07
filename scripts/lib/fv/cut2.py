#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of cutting motion 2.
    Cut until the hand reaches the table or if the robot collides with something.
  Usage: fv.cut2 [ARM]'''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else ct.robot.Arm
  knife_margin= 0.05
  cmargin= 1.1
  lw_xe= ct.GetAttr('wrist_'+LRToStrs(arm),'lx')
  ct.Run('addtable','add')
  ct.Run('viz','')
  xe0= ct.robot.FK(x_ext=lw_xe,arm=arm)

  #Target position
  xe1= copy.deepcopy(xe0)
  xe1[2]= ct.GetAttr('table','x')[2] + knife_margin

  #Approaching control
  shift_to_cut= ct.Load('ctrl').TApproachToX(ct)
  shift_to_cut.get_x_trg= lambda:xe1
  shift_to_cut.l_x_ext= lw_xe
  shift_to_cut.arm= arm
  shift_to_cut.init_callback= lambda:ct.Run('scene', 'make',[],cmargin)
  shift_to_cut.additional_check= lambda:(ct.Run('scene','isvalidq',arm))[0]
  shift_to_cut.exit_callback= lambda:ct.Run('scene', 'clear')

  shift_to_cut.Init()
  while shift_to_cut.Check():
    shift_to_cut.Step()
  shift_to_cut.Exit()

#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of cutting motion 3.
    Cut until the hand reaches the table.
    It stops if the robot collides with something
    or if the Argus' finger sensor detects too much force is applied.
  Usage: fv.cut3 [ARM]
    ARM is RIGHT in default.'''
def Run(ct,*args):
  arm= args[0] if len(args)>0 else RIGHT
  knife_margin= 0.05
  cmargin= 1.1

  if not ct.HasAttr(TMP,'vs_finger') or not ct.GetAttr(TMP,'vs_finger').running:
    CPrint(0,'fv.finger2 is not setup. Do you want to continue with setting it up?')
    if not ct.AskYesNo():
      return
    ct.Run('fv.finger2','setup')

  vs_finger= ct.GetAttr(TMP,'vs_finger')
  vsf_init= [vs_finger.force[RIGHT][0], vs_finger.force[LEFT][0]]
  vsf_force= lambda:-(vs_finger.force[RIGHT][0]-vsf_init[RIGHT])*(vs_finger.force[LEFT][0]-vsf_init[LEFT])
  is_danger= lambda: (vsf_force() > 5.0)
  vsf_rot= lambda: (abs(vs_finger.force[RIGHT][4])+abs(vs_finger.force[LEFT][4]))/2.0
  is_rotating= lambda: (vsf_rot() > 2.0)

  lw_xe= ct.GetAttr('wrist_'+LRToStrs(arm),'lx')
  ct.Run('addtable','add')
  ct.Run('viz','')
  xe0= ct.robot.FK(x_ext=lw_xe,arm=arm)

  ex0,ey0,ez0= RotToExyz(QToRot(xe0[3:]))

  #Target position-1
  xe1= copy.deepcopy(xe0)
  xe1[2]= ct.GetAttr('table','x')[2] + knife_margin

  #Approaching control
  shift_to_cut= ct.Load('ctrl').TApproachToX(ct)
  shift_to_cut.get_x_trg= lambda:xe1
  shift_to_cut.l_x_ext= lw_xe
  shift_to_cut.arm= arm
  shift_to_cut.max_speed= [0.05, 0.1]
  shift_to_cut.init_callback= lambda:ct.Run('scene', 'make',[],cmargin)
  shift_to_cut.additional_check= lambda:(ct.Run('scene','isvalidq',arm))[0]
  shift_to_cut.exit_callback= lambda:ct.Run('scene', 'clear')

  shift_to_cut.Init()
  while shift_to_cut.Check():
    if is_danger():
      CPrint(2,'Too much force is applied:',vsf_force())
      break
    if is_rotating():
      CPrint(2,'Rotating:',vsf_rot())
      break
    shift_to_cut.Step()
  shift_to_cut.Exit()

  #Slice control
  xe2= Vec(ct.robot.FK(x_ext=lw_xe,arm=arm))
  xe2[0:2]+= -0.04*Normalize(ex0[0:2])
  slice_cut= ct.Load('ctrl').TApproachToX(ct)
  slice_cut.get_x_trg= lambda:xe2
  slice_cut.l_x_ext= lw_xe
  slice_cut.arm= arm
  slice_cut.max_speed= [0.05, 0.1]

  slice_cut.Init()
  while slice_cut.Check():
    #if is_danger():
      #CPrint(2,'Too much force is applied:',vsf_force())
      #break
    slice_cut.Step()
  slice_cut.Exit()

  #Kickback control
  #xe3= Vec(ct.robot.FK(x_ext=lw_xe,arm=arm))
  #xe3[2]= xe0[2]
  kick_back= ct.Load('ctrl').TApproachToX(ct)
  kick_back.get_x_trg= lambda:xe0
  kick_back.l_x_ext= lw_xe
  kick_back.arm= arm
  kick_back.max_speed= [0.05, 0.1]

  kick_back.Init()
  while kick_back.Check():
    #if is_danger():
      #CPrint(2,'Too much force is applied:',vsf_force())
      #break
    kick_back.Step()
  kick_back.Exit()

#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of in-hand manipulation.
    Use fv.grasp before execiting this code.
  Usage:
    fv.inhand 'on' [, ARM]
      Turn on a in-hand manipulation thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.inhand 'off' [, ARM]
      Stop a in-hand manipulation thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.inhand
    fv.inhand 'clear'
      Stop all threads. Equivalent to:
        fv.inhand 'off' LEFT
        fv.inhand 'off' RIGHT'''
def ManipLoop(th_info, ct, arm):
  ct.Run('fv.ctrl_params')
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))

  side= 0 if fv_data.obj_area[0]>fv_data.obj_area[1] else 1
  get_theta= lambda: fv_data.obj_orientation[side]

  theta0= get_theta()
  target_angle= DegToRad(20.0)
  thread_cond= lambda: th_info.IsRunning() and not rospy.is_shutdown()
  while thread_cond():
    if abs(theta0-get_theta())>target_angle:
      print 'Done! target=', RadToDeg(target_angle)
      g_pos-= ct.GetAttr('fv_ctrl','min_gstep')[arm]
      ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=1.0, blocking=False)
      break

    #Open gripper until slip is detected
    g_pos= ct.robot.GripperPos(arm)
    while thread_cond() and sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])<0.05:
      g_pos+= ct.GetAttr('fv_ctrl','min_gstep')[arm]
      ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=1.0, blocking=False)
      for i in range(100):
        if abs(ct.robot.GripperPos(arm)-g_pos)<0.5*ct.GetAttr('fv_ctrl','min_gstep')[arm]:  break
        rospy.sleep(0.0001)
      for i in range(100):
        if sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>=0.05:  break
        rospy.sleep(0.001)
      g_pos= ct.robot.GripperPos(arm)

    #Close gripper to stop slip
    g_pos= ct.robot.GripperPos(arm)
    while thread_cond() and sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>0.05:
      g_pos-= ct.GetAttr('fv_ctrl','min_gstep')[arm]
      ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=1.0, blocking=False)
      for i in range(100):
        if abs(ct.robot.GripperPos(arm)-g_pos)<0.5*ct.GetAttr('fv_ctrl','min_gstep')[arm]:  break
        rospy.sleep(0.0001)
      g_pos= ct.robot.GripperPos(arm)

    print RadToDeg(theta0-get_theta()), RadToDeg(get_theta())

  #print 'Open the gripper and start object detection?'
  #if ct.AskYesNo():
    #ct.robot.OpenGripper(arm)
    ##Start object detection
    #ct.Run('fv.fv','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    if 'vs_inhand'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_inhand'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    CPrint(1,'Turn on:','vs_inhand'+LRToStrS(arm))
    ct.thread_manager.Add(name='vs_inhand'+LRToStrS(arm), target=lambda th_info: ManipLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    CPrint(2,'Turn off:','vs_inhand'+LRToStrS(arm))
    ct.thread_manager.Stop(name='vs_inhand'+LRToStrS(arm))

  elif command=='clear':
    CPrint(2,'Turn off:','vs_inhand'+LRToStrS(RIGHT))
    CPrint(2,'Turn off:','vs_inhand'+LRToStrS(LEFT))
    ct.thread_manager.Stop(name='vs_inhand'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_inhand'+LRToStrS(LEFT))


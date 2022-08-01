#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of slip-based holding.
    Closing gripper if slip is detected.
  Usage:
    fv.hold 'on' [, ARM]
      Turn on a holding thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.hold 'off' [, ARM]
      Stop a holding thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.hold
    fv.hold 'clear'
      Stop all threads. Equivalent to:
        fv.hold 'off' LEFT
        fv.hold 'off' RIGHT'''
def HoldLoop(th_info, ct, arm):
  ct.Run('fv.ctrl_params')
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))
  #slip_detect1= lambda: (sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>ct.GetAttr('fv_ctrl','hold_sensitivity_slip'),)
  slip_detect2= lambda: ((sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>ct.GetAttr('fv_ctrl','hold_sensitivity_slip'),
                          np.max(fv_data.d_obj_center_filtered)>ct.GetAttr('fv_ctrl','hold_sensitivity_oc'),
                          np.max(fv_data.d_obj_orientation_filtered)>ct.GetAttr('fv_ctrl','hold_sensitivity_oo'),
                          np.max(fv_data.d_obj_area_filtered)>ct.GetAttr('fv_ctrl','hold_sensitivity_oa')))

  #Stop object detection
  ct.Run('fv.fv','stop_detect_obj',arm)

  #if ct.robot.EndEff(arm).Is('DxlGripper'):
    #ct.robot.EndEff(arm).StartHolding()

  g_pos= ct.robot.GripperPos(arm)
  while th_info.IsRunning() and not rospy.is_shutdown():
    if any(slip_detect2()):
      print 'slip',slip_detect2(),rospy.Time.now().to_sec()
      #g_pos-= 0.0005 if arm==LEFT else 0.002
      g_pos-= ct.GetAttr('fv_ctrl','min_gstep')[arm]
      #ct.robot.MoveGripper(pos=g_pos, arm=arm, speed=100.0, blocking=False)
      #rospy.sleep(0.001)
      #g_pos= ct.robot.GripperPos(arm)
      ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=1.0, blocking=False)
      for i in range(100):  #100
        if abs(ct.robot.GripperPos(arm)-g_pos)<0.5*ct.GetAttr('fv_ctrl','min_gstep')[arm]:  break
        rospy.sleep(0.0001)
      g_pos= ct.robot.GripperPos(arm)
    else:
      rospy.sleep(0.001)
    rospy.sleep(0.04)  #0.02

  #if ct.robot.EndEff(arm).Is('DxlGripper'):
    #ct.robot.EndEff(arm).StopHolding()

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
    if 'vs_hold'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_hold'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    CPrint(1,'Turn on:','vs_hold'+LRToStrS(arm))
    ct.thread_manager.Add(name='vs_hold'+LRToStrS(arm), target=lambda th_info: HoldLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    CPrint(2,'Turn off:','vs_hold'+LRToStrS(arm))
    ct.thread_manager.Stop(name='vs_hold'+LRToStrS(arm))

  elif command=='clear':
    CPrint(2,'Turn off:','vs_hold'+LRToStrS(RIGHT))
    CPrint(2,'Turn off:','vs_hold'+LRToStrS(LEFT))
    ct.thread_manager.Stop(name='vs_hold'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_hold'+LRToStrS(LEFT))



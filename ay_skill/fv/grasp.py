#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of force-based grasping.
    Closing gripper until it finds a force.
  Usage:
    fv.grasp 'on' [, ARM]
      Turn on a grasping thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.grasp 'off' [, ARM]
      Stop a grasping thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.grasp
    fv.grasp 'clear'
      Stop all threads. Equivalent to:
        fv.grasp 'off' LEFT
        fv.grasp 'off' RIGHT'''

#Force change detector class.
class TForceChangeDetector(object):
  #Setup.  vs_finger: dictionary of observation that is expected to be updated by other thread.
  #th: threshold to stop, filter_len: temporal filter length.
  def __init__(self, vs_finger, th=3, filter_len=4, dstate_th=3):
    self.vs_finger= vs_finger
    self.dth= th
    self.smoothing_filter_len= filter_len
    self.dstate_th= dstate_th

  #Get discrete state for robust force change detection.
  def GetDState(self,side):
    #return vs_finger.dstate[side]
    return len([ds for ds in self.vs_finger.dstate_array[side] if ds>=self.dstate_th])

  #Initialize the internal state.
  def Init(self):
    #Correct some data for temporal filtering.
    #self.tm_last= rospy.Time.now()
    self.dstate_list= [[],[]]  #list of self.GetDState(0) and self.GetDState(1)
    for side in (0,1):
      self.dstate_list[side].append(self.GetDState(side))
    self.tm_last= max(self.vs_finger.tm_last_topic[:2])
    self.mean= Median
    self.is_initialized= False
    self.is_detected= False

  #Update dstate_list.
  def Update(self):
    if self.tm_last<max(self.vs_finger.tm_last_topic[:2]):
      for side in (0,1):
        self.dstate_list[side].append(self.GetDState(side))
        if len(self.dstate_list[side])>self.smoothing_filter_len:
          self.dstate_list[side].pop(0)
          if not self.is_initialized:
            self.dstate0= [self.mean(self.dstate_list[0]), self.mean(self.dstate_list[1])]
            self.is_initialized= True
      self.tm_last= max(self.vs_finger.tm_last_topic[:2])

      if self.is_initialized:
        #print self.dstate_list
        #print self.vs_finger.dstate_array
        #print self.mean(self.dstate_list[0])-self.dstate0[0], self.mean(self.dstate_list[1])-self.dstate0[1]
        if self.mean(self.dstate_list[0])>=self.dstate0[0]+self.dth or self.mean(self.dstate_list[1])>=self.dstate0[1]+self.dth:
          self.is_detected= True

  #Wait until the initialization is finished.
  def WaitForInitialization(self, dt=0.001, stop_callback=None):
    while not self.IsInitialized():
      if stop_callback is not None and stop_callback():  break
      self.Update()
      rospy.sleep(dt)

  #True if the initialization is finished (data for initial filter values are corrected).
  def IsInitialized(self):
    return self.is_initialized

  #True if force is detected.
  def IsDetected(self):
    return self.is_detected


def GraspLoop(th_info, ct, arm):
  ct.Run('fv.ctrl_params')
  vs_finger= ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm))
  #fa0= vs_finger.force_array[arm]
  #n_change= lambda: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(vs_finger.force_array[arm],fa0)])
  #dth= 7
  #dth= max(vs_finger.dstate[0],vs_finger.dstate[1]) + 4
  #continue_cond= lambda: vs_finger.dstate[0]<dth and vs_finger.dstate[1]<dth

  force_detector= TForceChangeDetector(vs_finger)
  force_detector.Init()

  #continue_cond= lambda: n_change()<5

  #Stop object detection
  ct.Run('fv.finger3','stop_detect_obj',arm)

  #g_pos= ct.robot.GripperPos(arm)
  #if arm==RIGHT:
    #while g_pos>0.0 and continue_cond():
      #ct.robot.MoveGripper(pos=g_pos, arm=arm, speed=10.0, blocking=True)
      #g_pos-= 0.001
  #elif arm==LEFT:
    #ct.robot.MoveGripper(pos=0.0, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=1.0, arm=arm)
    #while ct.robot.GripperPos(arm)>0.001 and continue_cond():
      #rospy.sleep(0.001)
    #ct.robot.grippers[LEFT].StopGripper()

  if ct.robot.EndEff(arm).Is('DxlGripper'):
    ct.robot.EndEff(arm).StartHolding()

  try:
    g_pos= ct.robot.GripperPos(arm)
    while th_info.IsRunning() and not rospy.is_shutdown():
      if g_pos<0.001 or force_detector.IsDetected():
        print 'Done'
        break

      if force_detector.IsInitialized():
        g_pos-= ct.GetAttr('fv_ctrl','min_gstep')[arm]
        ct.robot.MoveGripper(pos=g_pos, arm=arm, max_effort=ct.GetAttr('fv_ctrl','effort')[arm], speed=1.0, blocking=False)
        for i in range(100):
          if abs(ct.robot.GripperPos(arm)-g_pos)<0.5*ct.GetAttr('fv_ctrl','min_gstep')[arm]:  break
          rospy.sleep(0.0001)
        #rospy.sleep(0.1)
        g_pos= ct.robot.GripperPos(arm)

      force_detector.Update()

  finally:
    if ct.robot.EndEff(arm).Is('DxlGripper'):
      ct.robot.EndEff(arm).StopHolding()

  #print 'Open the gripper and start object detection?'
  #if ct.AskYesNo():
    #ct.robot.OpenGripper(arm)
    ##Start object detection
    #ct.Run('fv.finger3','start_detect_obj',arm)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    if 'vs_grasp'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_grasp'+LRToStrS(arm),'is already on'

    if not ct.HasAttr(TMP,'vs_finger'+LRToStrS(arm)) or not ct.GetAttr(TMP,'vs_finger'+LRToStrS(arm)).running:
      #CPrint(0,'fv.finger3 is not ready. Do you want to setup now?')
      #if not ct.AskYesNo():
        #return
      ct.Run('fv.finger3','setup',arm)

    print 'Turn on:','vs_grasp'+LRToStrS(arm)
    ct.thread_manager.Add(name='vs_grasp'+LRToStrS(arm), target=lambda th_info: GraspLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    print 'Turn off:','vs_grasp'+LRToStrS(arm)
    ct.thread_manager.Stop(name='vs_grasp'+LRToStrS(arm))

  elif command=='clear':
    print 'Turn off:','vs_grasp'+LRToStrS(RIGHT)
    print 'Turn off:','vs_grasp'+LRToStrS(LEFT)
    ct.thread_manager.Stop(name='vs_grasp'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_grasp'+LRToStrS(LEFT))


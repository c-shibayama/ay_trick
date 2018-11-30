#!/usr/bin/python
from core_tool import *
def Help():
  return '''Visualizing force change.
  Usage:
    fv.vizdiff 'on' [, ARM [, CTRL_TYPE]]
      Turn on the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      CTRL_TYPE: Control type ('pose','position','orientation'). Default: 'position'
    fv.vizdiff 'off' [, ARM]
      Stop the thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.vizdiff
    fv.vizdiff 'clear'
      Stop all threads. Equivalent to:
        fv.vizdiff 'off' LEFT
        fv.vizdiff 'off' RIGHT'''

def VizLoop(th_info, ct, arm):
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))
  m_vsf= ct.Load('fv.fv')

  pfa_scale= [[max(1.0, la.norm(p_f[2:])/2.0) for p_f in fv_data.posforce_array[side]] for side in range(2)]
  #pfa0= copy.deepcopy(fv_data.posforce_array)
  pfa0= [[[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(fv_data.posforce_array[side], pfa_scale[side])] for side in range(2)]
  pfa_filtered= copy.deepcopy(pfa0)
  ##FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
  #n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>0.9 else 0 for f,f0 in zip(fv_data.posforce_array[side],fa0[side])])

  def StepFilter(side):
    #We will consider only [fx,fz] since they are most reliable.
    th_f= 5.0
    #Each element in posforce_array is [x,z,fx,fy,fz]
    #pfa= fv_data.posforce_array[side]
    pfa= [[p_f[0],p_f[1],  p_f[2]/scale,p_f[3]/scale,p_f[4]/scale] for p_f,scale in zip(fv_data.posforce_array[side], pfa_scale[side])]
    pfa_filtered[side]= pfa
  warned= [False,False]
  def FDiff():
    force_array= []
    for side in (RIGHT,LEFT):
      diff_pfa= [[p_f_f[0],p_f_f[1], p_f_f[2]-p_f0[2],p_f_f[3]-p_f0[3],p_f_f[4]-p_f0[4]]
                for p_f0,p_f_f in zip(pfa0[side],pfa_filtered[side]) if p_f_f is not None]
      gpos= (-1.0,1.0)[side]*ct.robot.GripperPos(arm)
      force_array+= [p_f[2:] + np.cross([p_f[0],gpos,p_f[1]],p_f[2:]).tolist() for p_f in diff_pfa]
    f_diff= [sum([force[d] for force in force_array])/float(len(force_array)) for d in xrange(6)]
    return Vec(f_diff)

  #f0= Vec(fv_data.force[0]) + Vec(fv_data.force[1])
  #FDiff= lambda: (Vec(fv_data.force[0]) + Vec(fv_data.force[1])) - f0

  rate_adjuster= rospy.Rate(30)
  while th_info.IsRunning() and not rospy.is_shutdown():
    StepFilter(0)
    StepFilter(1)
    fd_l= 10.0*FDiff()
    m_vsf.VizForce(ct,fd_l,dstate=10,arm=arm,side=2)
    rate_adjuster.sleep()


def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    ctrl_type= args[1] if len(args)>1 else 'position'
    if 'vs_vizdiff'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_vizdiff'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    CPrint(1,'Turn on:','vs_vizdiff'+LRToStrS(arm))
    ct.thread_manager.Add(name='vs_vizdiff'+LRToStrS(arm), target=lambda th_info: VizLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    CPrint(2,'Turn off:','vs_vizdiff'+LRToStrS(arm))
    ct.thread_manager.Stop(name='vs_vizdiff'+LRToStrS(arm))

  elif command=='clear':
    CPrint(2,'Turn off:','vs_vizdiff'+LRToStrS(RIGHT))
    CPrint(2,'Turn off:','vs_vizdiff'+LRToStrS(LEFT))
    ct.thread_manager.Stop(name='vs_vizdiff'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_vizdiff'+LRToStrS(LEFT))


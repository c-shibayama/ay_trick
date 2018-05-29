#!/usr/bin/python
from core_tool import *
def Help():
  return '''Visualize Jacobian.
  Usage: test.viz_jaco'''

def VizLoop(th_info,ct):
  while th_info.IsRunning() and not rospy.is_shutdown():
    J= ct.robot.J()
    evals, evecs= la.eig(np.dot(J,J.T))
    idx= evals.argsort()[::-1]  #Sort by eigenvalue in decreasing order
    evecs= evecs[:,idx]
    evals= evals[idx]
    #print evals
    for i in range(3):
      x_w= ct.robot.FK()
      x_ji= Vec([0.0]*7)
      x_ji[:3]= x_w[:3]
      ex= ToList(evecs[:,i][:3])
      ex_norm= Norm(ex)
      ex= ex/ex_norm
      ey= GetOrthogonalAxisOf(ex, preferable=([0.0,1.0,0.0]))
      ez= np.cross(ex, ey)
      x_ji[3:]= RotToQ(ExyzToRot(ex,ey,ez))
      ct.viz.jaco.AddArrow(x_ji, scale=[0.02*evals[i],0.01,0.01], rgb=ct.viz.jaco.ICol(i), alpha=0.7, mid=2*i)
      x_ji[3:]= RotToQ(ExyzToRot(-ex,-ey,ez))
      ct.viz.jaco.AddArrow(x_ji, scale=[0.02*evals[i],0.01,0.01], rgb=ct.viz.jaco.ICol(i), alpha=0.7, mid=2*i+1)
    rospy.sleep(0.05)

def Run(ct,*args):
  if len(args)>0:
    command= args[0]
    args= args[1:]
  else:
    command= 'off'
    args= []

  if command=='on':
    ct.viz.jaco= TSimpleVisualizer(name_space='visualizer_estate')
    ct.viz.jaco.viz_frame= ct.robot.BaseFrame
    ct.thread_manager.Add(name='viz_jaco', target=lambda th_info,ct=ct: VizLoop(th_info,ct))

  elif command=='off':
    ct.thread_manager.Stop(name='viz_jaco')
    if 'jaco' in ct.viz:
      ct.viz.jaco.DeleteAllMarkers()

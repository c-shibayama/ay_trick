#!/usr/bin/python
from core_tool import *
def Help():
  return '''Visualize Jacobian (manipulability).
  Usage: test.viz_jaco2'''

def VizLoop(th_info,ct):
  while th_info.IsRunning() and not rospy.is_shutdown():
    J= ct.robot.J()
    x_w= ct.robot.FK()
    U,S,V=la.svd(ct.robot.J()[:3,:])
    for pm in (-1,1):
      for i in range(3):
        ex= (pm*U[:3,i]).ravel().tolist()[0]
        ey= GetOrthogonalAxisOf(ex, preferable=([0.0,1.0,0.0]))
        ez= np.cross(ex, ey)
        x_center= ToList(x_w[:3])+ToList(RotToQ(ExyzToRot(ex,ey,ez)))
        S_th= 0.12
        width= 0.01 if S[i]>S_th else 0.02
        col= ct.viz.jaco.ICol(i) if S[i]>S_th else ct.viz.jaco.ICol(3)
        ct.viz.jaco.AddArrow(x_center, scale=[0.2*S[i],width,width], rgb=col, alpha=0.7, mid=2*i+(pm+1)/2)
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
    ct.thread_manager.Add(name='viz_jaco2', target=lambda th_info,ct=ct: VizLoop(th_info,ct))

  elif command=='off':
    ct.thread_manager.Stop(name='viz_jaco2')
    if 'jaco' in ct.viz:
      ct.viz.jaco.DeleteAllMarkers()

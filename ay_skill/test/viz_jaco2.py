#!/usr/bin/python
from core_tool import *
def Help():
  return '''Visualize Jacobian.
  Usage: test.viz_jaco2'''

def VizLoop(th_info,ct):
  while th_info.IsRunning() and not rospy.is_shutdown():
    J= ct.robot.J()
    #print evals
    for i in range(6):
      dv= 0.05
      vx= ([dv,0,0,0,0,0],[0,dv,0,0,0,0],[0,0,dv,0,0,0],
           [-dv,0,0,0,0,0],[0,-dv,0,0,0,0],[0,0,-dv,0,0,0])[i]
      if ct.robot.DoF()>=6:
        dq= la.pinv(J)*MCVec(vx)
      else:  #e.g. Mikata Arm
        W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
        #W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.001,0.001,0.001]))
        #W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.0,0.0,0.0]))
        dq= la.pinv(W*J)*W*MCVec(vx)
      vx2= ToList(J*dq)
      x_w= ct.robot.FK()
      x_ji= Vec([0.0]*7)
      x_ji[:3]= x_w[:3]
      ex= vx2[:3]
      ex_norm= Norm(ex)
      ex= ex/ex_norm
      ey= GetOrthogonalAxisOf(ex, preferable=([0.0,1.0,0.0]))
      ez= np.cross(ex, ey)
      x_ji[3:]= RotToQ(ExyzToRot(ex,ey,ez))
      #print x_ji
      width= min(0.1,0.005+0.01*Norm(vx2[3:]))
      #width= 0.02
      ct.viz.jaco.AddArrow(x_ji, scale=[3.0*ex_norm,width,width], rgb=ct.viz.jaco.ICol(i), alpha=0.7, mid=i)
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

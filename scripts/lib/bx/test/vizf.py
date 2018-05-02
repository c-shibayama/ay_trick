#!/usr/bin/python
from core_tool import *
import baxter_core_msgs
def Help():
  return '''Visualize endeffector force.
  Usage: test.vizf'''

def CallbackEState(ct,msg):
  fx= msg.wrench.force.x
  fy= msg.wrench.force.y
  fz= msg.wrench.force.z
  x_w= ct.robot.FK(arm=RIGHT)
  x_f= Vec([0.0]*7)
  x_f[:3]= x_w[:3]
  ex= Vec([fx,fy,fz])
  f_norm= Norm(ex)
  ex= ex/f_norm
  ey= GetOrthogonalAxisOf(ex, preferable=([0.0,1.0,0.0]))
  ez= np.cross(ex, ey)
  x_f[3:]= RotToQ(ExyzToRot(ex,ey,ez))
  ct.viz.estate.AddArrow(x_f, scale=[0.02*f_norm,0.01,0.01], rgb=ct.viz.estate.ICol(1), alpha=0.7, mid=0)

def Run(ct,*args):
  if not ct.robot.Is('Baxter'):
    CPrint(4,'This program works only with Baxter.')
    return

  if len(args)>0:
    command= args[0]
    args= args[1:]
  else:
    command= 'clear'
    args= []

  if command=='setup':
    ct.viz.estate= TSimpleVisualizer(name_space='visualizer_estate')
    ct.viz.estate.viz_frame= ct.robot.BaseFrame
    res= (ct.AddSub('estate', '/robot/limb/right/endpoint_state', baxter_core_msgs.msg.EndpointState, lambda msg:CallbackEState(ct,msg)))

  elif command=='clear':
    ct.DelSub('estate')
    if 'estate' in ct.viz:
      ct.viz.estate.DeleteAllMarkers()

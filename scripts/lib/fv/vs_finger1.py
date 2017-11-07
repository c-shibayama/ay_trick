#!/usr/bin/python
from core_tool import *
roslib.load_manifest('ay_vision_msgs')
import ay_vision_msgs.msg
def Help():
  return '''Test fingertip visual sensors.
  Usage:
    fv.vs_finger1 'setup'
      Start to subscribe topics.
    fv.vs_finger1
    fv.vs_finger1 'clear'
      Stop to subscribe topics.
  '''

#Similar to a median of a position pos but
#return one of median(pos) and median(1-pos)
#that has a bigger absolute value.
def XMedian(array,pos=0.75):
  if len(array)==0:  return None
  a_sorted= copy.deepcopy(array)
  a_sorted.sort()
  a1= a_sorted[int(len(a_sorted)*pos)]
  a2= a_sorted[int(len(a_sorted)*(1.0-pos))]
  return a1 if abs(a1)>abs(a2) else a2

def VizForce(ct,force,side):
  viz= ct.viz.finger_force
  x_w= ct.robot.FK(x_ext=[0.0,(-0.05,0.05)[side],0.07, 0,0,0,1],arm=RIGHT)
  x_f= Vec([0.0]*7)
  x_f[:3]= x_w[:3]
  ex= Vec(force[:3])
  f_norm= Norm(ex)
  if f_norm<1.0e-6:
    viz.AddArrow(x_f, scale=[0.0,0.0,0.0], mid=0)
    return
  ex= ex/f_norm
  ey= GetOrthogonalAxisOf(ex, preferable=([0.0,1.0,0.0]), fault=[1.0,0.0,0.0])
  ez= np.cross(ex, ey)
  x_f[3:]= RotToQ(ExyzToRot(ex,ey,ez))
  rgb= [(force[3] if force[3]>0.0 else 0.0), 0.5, (-force[3] if force[3]<0.0 else 0.0)]
  width= 0.005+0.005*abs(force[3])
  viz.AddArrow(x_f, scale=[0.02*f_norm,width,width], rgb=rgb, alpha=0.7, mid=side)

def BlobMoves(ct,l,msg,side):
  cx= msg.width/2
  cy= msg.height/2
  div= (msg.width+msg.height)/2
  def convert(mv):
    r= Vec((mv.Pox-cx, mv.Poy-cy))/div
    fxy= Vec((mv.DPx, mv.DPy))
    fz= max(0.0,mv.DS)
    tau_z= np.cross(r,fxy)
    #return int(fxy[0]/1.0), int(fxy[1]/1.0), int(fz/0.4), int(tau_z/0.3)
    #return float(fxy[0]/1.0), float(fxy[1]/1.0), float(fz/0.4), float(tau_z/0.3)
    if side==RIGHT:
      return +float(fxy[0]/1.0), -float(fz/0.4), -float(fxy[1]/1.0), -float(tau_z/0.3)
    elif side==LEFT:
      return -float(fxy[0]/1.0), +float(fz/0.4), -float(fxy[1]/1.0), +float(tau_z/0.3)
  force_array= [convert(mv) for mv in msg.data]
  #print force_array
  l.seq[side].append(force_array)
  if len(l.seq[side])>20:  l.seq[side].pop()
  force= [XMedian([force[d] for force in force_array],0.8) for d in xrange(4)]
  #print force
  VizForce(ct,force,side)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='setup':
    ct.viz.finger_force= TSimpleVisualizer(name_space='visualizer_estate')
    ct.viz.finger_force.viz_frame= ct.robot.BaseFrame
    l= TContainer(debug=True)
    l.seq= [[],[]]
    ct.AddSub('vs_blob_moves_usbcam2f1_r', '/visual_skin_node/blob_moves_usbcam2f1_r', ay_vision_msgs.msg.BlobMoves, lambda msg:BlobMoves(ct,l,msg,RIGHT))
    ct.AddSub('vs_blob_moves_usbcam2f1_l', '/visual_skin_node/blob_moves_usbcam2f1_l', ay_vision_msgs.msg.BlobMoves, lambda msg:BlobMoves(ct,l,msg,LEFT))

  elif command=='clear':
    ct.DelSub('vs_blob_moves_usbcam2f1_r')
    ct.DelSub('vs_blob_moves_usbcam2f1_l')


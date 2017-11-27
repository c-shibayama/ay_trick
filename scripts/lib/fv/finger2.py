#!/usr/bin/python
from core_tool import *
roslib.load_manifest('ay_vision_msgs')
import ay_vision_msgs.msg
def Help():
  return '''Test fingertip visual sensors ver.2.
  Usage:
    fv.finger2 'setup'
      Start to subscribe topics.
    fv.finger2 'rec'
      Start/stop recording.
    fv.finger2
    fv.finger2 'clear'
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
  x_e= ct.robot.FK(x_ext=ct.GetAttr('wrist_r','lx'),arm=RIGHT)
  def VizVector(ct,vec,width,col,mid):
    x_f= Vec([0.0]*7)
    x_f[:3]= x_w[:3]
    ex= Vec(vec)
    f_norm= Norm(ex)
    if f_norm<1.0e-6:
      viz.AddArrow(x_f, scale=[0.0,0.0,0.0], mid=mid)
      return
    ex= ex/f_norm
    ey= GetOrthogonalAxisOf(ex, preferable=([0.0,1.0,0.0]), fault=[1.0,0.0,0.0])
    ez= np.cross(ex, ey)
    x_f[3:]= RotToQ(ExyzToRot(ex,ey,ez))
    viz.AddArrow(x_f, scale=[0.02*f_norm,width,width], rgb=col, alpha=0.7, mid=mid)
  VizVector(ct,Transform(x_e[3:],force[:3]),width=0.010,col=[1.0,0.0,0.0],mid=2*side+0)
  VizVector(ct,Transform(x_e[3:],force[3:]),width=0.005,col=[0.0,1.0,0.0],mid=2*side+1)

def BlobMoves(ct,l,msg,side):
  filter_len= 20
  cx= msg.width/2
  cy= msg.height/2
  div= (msg.width+msg.height)/2
  def convert(mv):
    fscale= [1.0,1.0,2.5]
    tscale= 2.0
    rxy= ((mv.Pox-cx)/div, (mv.Poy-cy)/div)
    fxy= (mv.DPx, mv.DPy)
    fz= max(0.0,mv.DS)
    if side==RIGHT:
      f= [+(fxy[0]*fscale[0]), -(fz*fscale[2]), -(fxy[1]*fscale[1])]
      r= [+rxy[0], 0.0, -rxy[1]]
    elif side==LEFT:
      f= [-(fxy[0]*fscale[0]), +(fz*fscale[2]), -(fxy[1]*fscale[1])]
      r= [-rxy[0], 0.0, -rxy[1]]
    tau= np.cross(r,f)*tscale
    return f+tau.tolist()
  force_array= [convert(mv) for mv in msg.data]
  #print force_array
  l.seq[side].append(force_array)
  if len(l.seq[side])>filter_len:  l.seq[side].pop()
  force= [XMedian([force[d] for force in force_array],0.8) for d in xrange(6)]
  l.force[side]= force
  #print force
  VizForce(ct,force,side)
  #Callback(l)
  def serialize(data):
    return ' '.join((ToStr((mv.Pox,mv.Poy,mv.So,mv.DPx,mv.DPy,mv.DS)) for mv in msg.data))
  if l.recording:
    t_curr= rospy.Time.now().to_nsec()
    l.fp_avr[side].write('{time} {force}\n'.format(
                      time=t_curr,
                      force=ToStr(force)))
    l.fp_all[side].write('{time} {width} {height} {data}\n'.format(
                      time=t_curr,
                      width=msg.width,height=msg.height,
                      data=serialize(msg.data)))

def Callback(l):
  #offset= [-1.0,4.0]
  offset= [0.0,0.0]
  if None not in l.force:
    power= -(l.force[RIGHT][0]-offset[RIGHT])*(l.force[LEFT][0]-offset[LEFT])
    if power>20.0:
      CPrint(4,'Danger!!!',power)
    #else:
      #CPrint(1,'Safe',power)
    rot= (abs(l.force[RIGHT][4])+abs(l.force[LEFT][4]))/2.0
    if rot>2.0:
      CPrint(4,'Rotating!!!',rot)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  def StartStopRecording(ct,rid=None):
    if not ct.HasAttr(TMP,'vs_finger'):
      print 'Error: Run setup first'
      return
    l= ct.GetAttr(TMP,'vs_finger')
    if l.recording:
      l.recording= False
      for fp in l.fp_avr+l.fp_all:
        fp.close()
      print 'Stop recording to',l.file_name_avr,l.file_name_all
    else:
      s= '-{rid}-'.format(rid=rid) if rid is not None else ''
      l.file_name_avr= [ct.LogFileName('vsfingerAvrR'+s),ct.LogFileName('vsfingerAvrL'+s)]
      l.file_name_all= [ct.LogFileName('vsfingerAllR'+s),ct.LogFileName('vsfingerAllL'+s)]
      l.fp_avr= map(OpenW, l.file_name_avr)
      l.fp_all= map(OpenW, l.file_name_all)
      l.recording= True
      print 'Start recording to',l.file_name_avr,l.file_name_all

  if command=='setup':
    ct.viz.finger_force= TSimpleVisualizer(name_space='visualizer_estate')
    ct.viz.finger_force.viz_frame= ct.robot.BaseFrame
    l= TContainer(debug=True)
    l.seq= [[],[]]
    l.force= [None,None]
    l.recording= False
    l.running= True
    ct.SetAttr(TMP,'vs_finger', l)
    ct.AddSub('vs_blob_moves_usbcam2f1_r', '/visual_skin_node/blob_moves_usbcam2f1_r', ay_vision_msgs.msg.BlobMoves, lambda msg:BlobMoves(ct,l,msg,RIGHT))
    ct.AddSub('vs_blob_moves_usbcam2f1_l', '/visual_skin_node/blob_moves_usbcam2f1_l', ay_vision_msgs.msg.BlobMoves, lambda msg:BlobMoves(ct,l,msg,LEFT))

  elif command=='rec':
    rid= args[0] if len(args)>0 else None
    StartStopRecording(ct,rid)

  elif command=='clear':
    if ct.HasAttr(TMP,'vs_finger'):
      if ct.GetAttr(TMP,'vs_finger').recording:
        StartStopRecording(ct)
      ct.GetAttr(TMP,'vs_finger').running= False
    ct.DelSub('vs_blob_moves_usbcam2f1_r')
    ct.DelSub('vs_blob_moves_usbcam2f1_l')


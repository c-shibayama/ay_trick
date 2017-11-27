#!/usr/bin/python
from core_tool import *
import std_srvs.srv
roslib.load_manifest('ay_vision_msgs')
import ay_vision_msgs.msg
import ay_vision_msgs.srv
def Help():
  return '''Test fingertip visual skin (Optical Skin) sensors ver.3.
    This uses visual_skin_node2 that has an improved marker tracker
    and proximity vision (object detection, tracking, and movement detection).
  Usage:
    fv.finger3 'setup' [, ARM1 [, ARM2]]
      Start to subscribe topics, setup services.
      ARM*: LEFT, RIGHT, or 'all' (both arms). Default: 'all'
    fv.finger3 'rec' [, RID [, ARM1 [, ARM2]]]
      Start/stop recording.
      RID: Recoding ID.
      ARM*: LEFT, RIGHT, or 'all' (both arms). Default: 'all'
    fv.finger3
    fv.finger3 'clear' [, ARM1 [, ARM2]]
      Stop to subscribe topics.
      ARM*: LEFT, RIGHT, or 'all' (both arms). Default: 'all'
    fv.finger3 'frame_skip', SKIP [, ARM1 [, ARM2]]
      Set frame-skip to SKIP.
      SKIP: Frames to be skipped. 0: No skip.
      ARM*: LEFT, RIGHT, or 'all' (both arms). Default: 'all'
    fv.finger3 'clear_obj' [, ARM1 [, ARM2]]
      Clear detected object models.
      ARM*: LEFT, RIGHT, or 'all' (both arms). Default: 'all'
    fv.finger3 'stop_detect_obj' [, ARM1 [, ARM2]]
      Stop detecting object.
      ARM*: LEFT, RIGHT, or 'all' (both arms). Default: 'all'
    fv.finger3 'start_detect_obj' [, ARM1 [, ARM2]]
      Start detecting object.
      ARM*: LEFT, RIGHT, or 'all' (both arms). Default: 'all'
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

def VizForce(ct,force,dstate,arm,side):
  viz= ct.viz.finger_force
  dy= (-0.05,0.05,0.0)[side]
  dz= [0.12,0.20][arm]
  wrist= ['wrist_r','wrist_l'][arm]
  x_w= ct.robot.FK(x_ext=[0.0,dy,dz, 0,0,0,1],arm=arm)
  x_e= ct.robot.FK(x_ext=ct.GetAttr(wrist,'lx'),arm=arm)
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
  c= min(dstate,5.0)/5.0
  VizVector(ct,Transform(x_e[3:],force[:3]),width=0.010,col=[c,0.0,0.0],mid=4*arm+2*side+0)
  VizVector(ct,Transform(x_e[3:],force[3:]),width=0.005,col=[0.0,c,0.0],mid=4*arm+2*side+1)

def BlobMoves(ct,l,msg,side):
  l.tm_last_topic[side]= rospy.Time.now()
  #filter_len= 20
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  def convert_raw(mv):
    fscale= [1.0,1.0,1.0]
    rxy= ((mv.Pox-cx)/div, (mv.Poy-cy)/div)
    fxy= (mv.DPx, mv.DPy)
    fz= la.norm(fxy) # max(0.0,mv.DS)
    if side==RIGHT:
      f= [+(fxy[0]*fscale[0]), -(fz*fscale[2]), -(fxy[1]*fscale[1])]
      p= [+rxy[0], -rxy[1]]
    elif side==LEFT:
      f= [-(fxy[0]*fscale[0]), +(fz*fscale[2]), -(fxy[1]*fscale[1])]
      p= [-rxy[0], -rxy[1]]
    return p+f
  def convert_wrench(p_f):
    p,f= p_f[:2], p_f[2:]
    tscale= 1.0
    tau= np.cross([p[0],0.0,p[1]],f)*tscale
    return f+tau.tolist()
  def convert_dstate(p_f):
    p,f= p_f[:2], p_f[2:]
    fz= abs(f[1])
    #TODO: Learn this categorization.
    if fz<0.8:  dstate= 0
    elif fz<1.8:  dstate= 1
    elif fz<2.5:  dstate= 3
    else:  dstate= 5
    return dstate
  posforce_array= [convert_raw(mv) for mv in msg.data]
  force_array= [convert_wrench(p_f) for p_f in posforce_array]
  dstate_array= [convert_dstate(p_f) for p_f in posforce_array]
  ##print force_array
  #l.seq[side].append(force_array)
  #if len(l.seq[side])>filter_len:  l.seq[side].pop()
  ##force= [XMedian([force[d] for force in force_array],0.8) for d in xrange(6)]
  force= [sum([force[d] for force in force_array])/float(len(force_array)) for d in xrange(6)]
  dstate= sum(dstate_array)
  l.posforce_array[side]= posforce_array
  l.force_array[side]= force_array
  l.dstate_array[side]= dstate_array
  l.force[side]= force
  l.dstate[side]= dstate
  #print force
  #print dstate, force_array[3][1]
  VizForce(ct,force,dstate,l.arm,side)
  if ct.callback.vs_finger_bm[LRToStrS(l.arm)][side] is not None:
    ct.callback.vs_finger_bm[LRToStrS(l.arm)][side](ct, l, side)
  #Callback(l)
  def serialize(data):
    return ' '.join((ToStr((mv.Pox,mv.Poy,mv.So,mv.DPx,mv.DPy,mv.DS)) for mv in msg.data))
  if l.recording:
    t_curr= rospy.Time.now().to_nsec()
    l.fp_avr[side].write('{time} {force} {x}\n'.format(
                      time=t_curr,
                      force=ToStr(force),
                      x=ToStr(ct.robot.FK(arm=l.arm))))
    l.fp_all[side].write('{time} {width} {height} {data}\n'.format(
                      time=t_curr,
                      width=msg.width,height=msg.height,
                      data=serialize(msg.data)))

def ProxVision(ct,l,msg,side):
  l.tm_last_topic[2+side]= rospy.Time.now()
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  diva= float(msg.width*msg.height)
  #Object and movement detection (shrunken form, e.g. 3x3 mat)
  l.obj_s[side]= [i/diva for i in msg.ObjS]  #FIXME: Do not divide by diva since MvS elements are already normalized by the local area!!! 255 would be better
  l.mv_s[side]= [100.0*i/diva for i in msg.MvS]  #FIXME: Do not divide by diva since MvS elements are already normalized by the local area!!! 255 would be better
  #Get object center and orientation from moment
  m00,m10,m01= msg.ObjM_m[:3]
  mu20,mu11,mu02= msg.ObjM_mu[:3]
  if m00>0.0:
    l.obj_center[side]= [(m10/m00-cx)/div, (m01/m00-cy)/div]
    l.obj_orientation[side]= 0.5*math.atan2(2.0*mu11/m00, (mu20/m00-mu02/m00))
    l.obj_area[side]= m00/diva
  else:
    l.obj_center[side]= [0.0, 0.0]
    l.obj_orientation[side]= 0.0
    l.obj_area[side]= 0.0
  #if l.obj_area[side]>0.2:
    #print LRToStr(l.arm), side, l.obj_area[side], l.obj_orientation[side], l.obj_center[side], sum(l.mv_s[side])
  if ct.callback.vs_finger_pv[LRToStrS(l.arm)][side] is not None:
    ct.callback.vs_finger_pv[LRToStrS(l.arm)][side](ct, l, side)
  if l.recording:
    t_curr= rospy.Time.now().to_nsec()
    l.fp_pv1[side].write('{time} {c} {o} {a} {sumo} {summ}\n'.format(
      time=t_curr,
      c=ToStr(l.obj_center[side]),
      o=l.obj_orientation[side],
      a=l.obj_area[side],
      sumo=sum(l.obj_s[side]),
      summ=sum(l.mv_s[side])))
    l.fp_pv2[side].write('{time} {obj_s} {mv_s}\n'.format(
      time=t_curr,
      obj_s=ToStr(l.obj_s[side]),
      mv_s=ToStr(l.mv_s[side])))

#def Callback(l):
  ##offset= [-1.0,4.0]
  #offset= [0.0,0.0]
  #if None not in l.force:
    #power= -(l.force[RIGHT][0]-offset[RIGHT])*(l.force[LEFT][0]-offset[LEFT])
    #if power>20.0:
      #CPrint(4,'Danger!!!',power)
    ##else:
      ##CPrint(1,'Safe',power)
    #rot= (abs(l.force[RIGHT][4])+abs(l.force[LEFT][4]))/2.0
    #if rot>2.0:
      #CPrint(4,'Rotating!!!',rot)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  rqdxl= (ct.robot.Is('RobotiqNB') or ct.robot.Is('DxlGripper'))

  def StartStopRecording(ct,arm,rid=None):
    vs= LRToStrS(arm)
    if not ct.HasAttr(TMP,'vs_finger'+vs):
      print 'Error: Setup first'
      return
    l= ct.GetAttr(TMP,'vs_finger'+vs)
    if l.recording:
      l.recording= False
      for fp in l.fp_avr+l.fp_all+l.fp_pv1+l.fp_pv2:
        fp.close()
      CPrint(1,'Stop recording to')
      print ' ',l.file_name_avr
      print ' ',l.file_name_all
      print ' ',l.file_name_pv1
      print ' ',l.file_name_pv2
    else:
      s= '-{rid}-'.format(rid=rid) if rid is not None else ''
      l.file_name_avr= [ct.LogFileName('vsf'+vs+'avrR'+s),ct.LogFileName('vsf'+vs+'avrL'+s)]
      l.file_name_all= [ct.LogFileName('vsf'+vs+'allR'+s),ct.LogFileName('vsf'+vs+'allL'+s)]
      l.file_name_pv1= [ct.LogFileName('vsf'+vs+'pv1R'+s),ct.LogFileName('vsf'+vs+'pv1L'+s)]
      l.file_name_pv2= [ct.LogFileName('vsf'+vs+'pv2R'+s),ct.LogFileName('vsf'+vs+'pv2L'+s)]
      l.fp_avr= map(OpenW, l.file_name_avr)
      l.fp_all= map(OpenW, l.file_name_all)
      l.fp_pv1= map(OpenW, l.file_name_pv1)
      l.fp_pv2= map(OpenW, l.file_name_pv2)
      l.recording= True
      CPrint(2,'Start recording to')
      print ' ',l.file_name_avr
      print ' ',l.file_name_all
      print ' ',l.file_name_pv1
      print ' ',l.file_name_pv2

  if command=='setup':
    if len(args)==0:  args= ['all']
    arms= set(sum([[RIGHT,LEFT] if a=='all' else [a] for a in args],[]))
    if rqdxl:
      arms= set(RIGHT if a==LEFT else a for a in arms)
    ct.viz.finger_force= TSimpleVisualizer(name_space='visualizer_estate')
    ct.viz.finger_force.viz_frame= ct.robot.BaseFrame
    for arm in arms:
      vs= LRToStrS(arm)
      ct.SetAttr(TMP,'vs_finger'+vs, TContainer(debug=True))
      if 'vs_finger_bm' not in ct.callback:  ct.callback.vs_finger_bm= TContainer(debug=True)
      ct.callback.vs_finger_bm[vs]= [None,None]  #Callback in BlobMoves [vs][Right/Left]
      if 'vs_finger_pv' not in ct.callback:  ct.callback.vs_finger_pv= TContainer(debug=True)
      ct.callback.vs_finger_pv[vs]= [None,None]  #Callback in ProxVision [vs][Right/Left]
      l= ct.GetAttr(TMP,'vs_finger'+vs)
      l.arm= arm
      l.vs_finger= 'vs_finger'+vs
      l.seq= [[],[]]
      l.posforce_array= [None,None]
      l.force_array= [None,None]
      l.dstate_array= [None,None]
      l.force= [None,None]
      l.dstate= [0,0]
      l.obj_s= [None,None]
      l.mv_s= [None,None]
      l.obj_center= [None,None]
      l.obj_orientation= [None,None]
      l.obj_area= [None,None]
      l.tm_last_topic= [None,None,None,None]  #ros time of last topic receiving. blob_moves-r,l, prox_vision-r,l
      l.recording= False
      l.running= True
      if arm==RIGHT:
        if ct.robot.Is('RobotiqNB'):
          ct.AddSub('vsay1_blob_moves_usbcam2fay12_r', '/visual_skin_node_ay10r/blob_moves_usbcam2fay10a_r',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,RIGHT))
          ct.AddSub('vsay1_blob_moves_usbcam2fay12_l', '/visual_skin_node_ay10l/blob_moves_usbcam2fay10a_l',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,LEFT))
          ct.AddSub('vsay1_prox_vision_usbcam2fay12_r', '/visual_skin_node_ay10r/prox_vision_usbcam2fay10a_r',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,RIGHT))
          ct.AddSub('vsay1_prox_vision_usbcam2fay12_l', '/visual_skin_node_ay10l/prox_vision_usbcam2fay10a_l',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,LEFT))
          ct.AddSrvP('vsay1r_clear_obj', '/visual_skin_node_ay10r/clear_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1r_set_frame_skip', '/visual_skin_node_ay10r/set_frame_skip', ay_vision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1r_start_detect_obj', '/visual_skin_node_ay10r/start_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1r_stop_detect_obj', '/visual_skin_node_ay10r/stop_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_clear_obj', '/visual_skin_node_ay10l/clear_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_set_frame_skip', '/visual_skin_node_ay10l/set_frame_skip', ay_vision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_start_detect_obj', '/visual_skin_node_ay10l/start_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_stop_detect_obj', '/visual_skin_node_ay10l/stop_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
        elif ct.robot.Is('DxlGripper'):
          ct.AddSub('vsay1_blob_moves_usbcam2fay12_r', '/visual_skin_node_ay11r/blob_moves_usbcam2fay11a_r',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,RIGHT))
          ct.AddSub('vsay1_blob_moves_usbcam2fay12_l', '/visual_skin_node_ay11l/blob_moves_usbcam2fay11a_l',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,LEFT))
          ct.AddSub('vsay1_prox_vision_usbcam2fay12_r', '/visual_skin_node_ay11r/prox_vision_usbcam2fay11a_r',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,RIGHT))
          ct.AddSub('vsay1_prox_vision_usbcam2fay12_l', '/visual_skin_node_ay11l/prox_vision_usbcam2fay11a_l',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,LEFT))
          ct.AddSrvP('vsay1r_clear_obj', '/visual_skin_node_ay11r/clear_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1r_set_frame_skip', '/visual_skin_node_ay11r/set_frame_skip', ay_vision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1r_start_detect_obj', '/visual_skin_node_ay11r/start_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1r_stop_detect_obj', '/visual_skin_node_ay11r/stop_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_clear_obj', '/visual_skin_node_ay11l/clear_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_set_frame_skip', '/visual_skin_node_ay11l/set_frame_skip', ay_vision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_start_detect_obj', '/visual_skin_node_ay11l/start_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1l_stop_detect_obj', '/visual_skin_node_ay11l/stop_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
        else:
          ct.AddSub('vsay1_blob_moves_usbcam2fay12_r', '/visual_skin_node_ay1/blob_moves_usbcam2fay12_r',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,RIGHT))
          ct.AddSub('vsay1_blob_moves_usbcam2fay12_l', '/visual_skin_node_ay1/blob_moves_usbcam2fay12_l',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,LEFT))
          ct.AddSub('vsay1_prox_vision_usbcam2fay12_r', '/visual_skin_node_ay1/prox_vision_usbcam2fay12_r',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,RIGHT))
          ct.AddSub('vsay1_prox_vision_usbcam2fay12_l', '/visual_skin_node_ay1/prox_vision_usbcam2fay12_l',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,LEFT))
          ct.AddSrvP('vsay1_clear_obj', '/visual_skin_node_ay1/clear_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1_set_frame_skip', '/visual_skin_node_ay1/set_frame_skip', ay_vision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1_start_detect_obj', '/visual_skin_node_ay1/start_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay1_stop_detect_obj', '/visual_skin_node_ay1/stop_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
      elif arm==LEFT:
        if rqdxl:
          raise Exception('This should be a bug.')
        else:
          ct.AddSub('vsay2_blob_moves_usbcam2fay22_r', '/visual_skin_node_ay2/blob_moves_usbcam2fay22_r',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,RIGHT))
          ct.AddSub('vsay2_blob_moves_usbcam2fay22_l', '/visual_skin_node_ay2/blob_moves_usbcam2fay22_l',
                  ay_vision_msgs.msg.BlobMoves, lambda msg,l=l:BlobMoves(ct,l,msg,LEFT))
          ct.AddSub('vsay2_prox_vision_usbcam2fay22_r', '/visual_skin_node_ay2/prox_vision_usbcam2fay22_r',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,RIGHT))
          ct.AddSub('vsay2_prox_vision_usbcam2fay22_l', '/visual_skin_node_ay2/prox_vision_usbcam2fay22_l',
                  ay_vision_msgs.msg.ProxVision, lambda msg,l=l:ProxVision(ct,l,msg,LEFT))
          ct.AddSrvP('vsay2_clear_obj', '/visual_skin_node_ay2/clear_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay2_set_frame_skip', '/visual_skin_node_ay2/set_frame_skip', ay_vision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay2_start_detect_obj', '/visual_skin_node_ay2/start_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
          ct.AddSrvP('vsay2_stop_detect_obj', '/visual_skin_node_ay2/stop_detect_obj', std_srvs.srv.Empty, persistent=False, time_out=3.0)
      if arm==RIGHT:
        if rqdxl:
          ct.srvp.vsay1r_start_detect_obj(std_srvs.srv.EmptyRequest())
          ct.srvp.vsay1l_start_detect_obj(std_srvs.srv.EmptyRequest())
          ct.srvp.vsay1r_clear_obj(std_srvs.srv.EmptyRequest())
          ct.srvp.vsay1l_clear_obj(std_srvs.srv.EmptyRequest())
        else:
          ct.srvp.vsay1_start_detect_obj(std_srvs.srv.EmptyRequest())
          ct.srvp.vsay1_clear_obj(std_srvs.srv.EmptyRequest())
      elif arm==LEFT:
        ct.srvp.vsay2_start_detect_obj(std_srvs.srv.EmptyRequest())
        ct.srvp.vsay2_clear_obj(std_srvs.srv.EmptyRequest())

  elif command=='rec':
    rid= args[0] if len(args)>0 else None
    if len(args[1:])==0:  args= [None,'all']
    arms= set(sum([[RIGHT,LEFT] if a=='all' else [a] for a in args[1:]],[]))
    for arm in arms:
      StartStopRecording(ct,arm,rid)

  elif command=='clear':
    if len(args)==0:  args= ['all']
    arms= set(sum([[RIGHT,LEFT] if a=='all' else [a] for a in args],[]))
    for arm in arms:
      vs= LRToStrS(arm)
      print 'Stopping','vs_finger'+vs
      if ct.HasAttr(TMP,'vs_finger'+vs):
        if ct.GetAttr(TMP,'vs_finger'+vs).recording:
          StartStopRecording(ct,arm)
        ct.GetAttr(TMP,'vs_finger'+vs).running= False
      if 'vs_finger_bm' in ct.callback:  ct.callback.vs_finger_bm[vs]= [None,None]
      if 'vs_finger_pv' in ct.callback:  ct.callback.vs_finger_pv[vs]= [None,None]
      if arm==RIGHT:
        ct.DelSub('vsay1_blob_moves_usbcam2fay12_r')
        ct.DelSub('vsay1_blob_moves_usbcam2fay12_l')
        ct.DelSub('vsay1_prox_vision_usbcam2fay12_r')
        ct.DelSub('vsay1_prox_vision_usbcam2fay12_l')
      elif arm==LEFT:
        ct.DelSub('vsay2_blob_moves_usbcam2fay22_r')
        ct.DelSub('vsay2_blob_moves_usbcam2fay22_l')
        ct.DelSub('vsay2_prox_vision_usbcam2fay22_r')
        ct.DelSub('vsay2_prox_vision_usbcam2fay22_l')

  elif command=='frame_skip':
    skip= args[0] if len(args)>0 else None
    if len(args[1:])==0:  args= [None,'all']
    arms= set(sum([[RIGHT,LEFT] if a=='all' else [a] for a in args[1:]],[]))
    set_frame_skip_req= ay_vision_msgs.srv.SetInt32Request()
    set_frame_skip_req.data= skip
    for arm in arms:
      if arm==RIGHT:   ct.srvp.vsay1_set_frame_skip(set_frame_skip_req) if not rqdxl else (ct.srvp.vsay1r_set_frame_skip(set_frame_skip_req), ct.srvp.vsay1l_set_frame_skip(set_frame_skip_req))
      elif arm==LEFT:  ct.srvp.vsay2_set_frame_skip(set_frame_skip_req)

  elif command=='clear_obj':
    if len(args)==0:  args= ['all']
    arms= set(sum([[RIGHT,LEFT] if a=='all' else [a] for a in args],[]))
    for arm in arms:
      if arm==RIGHT:   ct.srvp.vsay1_clear_obj(std_srvs.srv.EmptyRequest()) if not rqdxl else (ct.srvp.vsay1r_clear_obj(std_srvs.srv.EmptyRequest()), ct.srvp.vsay1l_clear_obj(std_srvs.srv.EmptyRequest()))
      elif arm==LEFT:  ct.srvp.vsay2_clear_obj(std_srvs.srv.EmptyRequest())

  elif command=='stop_detect_obj':
    if len(args)==0:  args= ['all']
    arms= set(sum([[RIGHT,LEFT] if a=='all' else [a] for a in args],[]))
    for arm in arms:
      if arm==RIGHT:   ct.srvp.vsay1_stop_detect_obj(std_srvs.srv.EmptyRequest()) if not rqdxl else (ct.srvp.vsay1r_stop_detect_obj(std_srvs.srv.EmptyRequest()), ct.srvp.vsay1l_stop_detect_obj(std_srvs.srv.EmptyRequest()))
      elif arm==LEFT:  ct.srvp.vsay2_stop_detect_obj(std_srvs.srv.EmptyRequest())

  elif command=='start_detect_obj':
    if len(args)==0:  args= ['all']
    arms= set(sum([[RIGHT,LEFT] if a=='all' else [a] for a in args],[]))
    for arm in arms:
      if arm==RIGHT:   ct.srvp.vsay1_start_detect_obj(std_srvs.srv.EmptyRequest()) if not rqdxl else (ct.srvp.vsay1r_start_detect_obj(std_srvs.srv.EmptyRequest()), ct.srvp.vsay1l_start_detect_obj(std_srvs.srv.EmptyRequest()))
      elif arm==LEFT:  ct.srvp.vsay2_start_detect_obj(std_srvs.srv.EmptyRequest())


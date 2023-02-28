#!/usr/bin/python
from core_tool import *
import std_srvs.srv
roslib.load_manifest('fingervision_msgs')
import fingervision_msgs.msg
import fingervision_msgs.srv
def Help():
  return '''FingerVision utility.
  Usage:
    fv.fv 'on' [, ARM1 [, ARM2]] [, FV_NAMES [, NODE_NAMES]]
    fv.fv 'setup' [, ARM1 [, ARM2]] [, FV_NAMES [, NODE_NAMES]]
      Start to subscribe topics, setup services.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
      FV_NAMES: FV camera names (dict of dict, {arm:{LEFT/RIGHT:fv_l/fv_r} }.
      NODE_NAMES: FV node names (dict of dict, {arm:{LEFT/RIGHT:fv_l_node/fv_r_node} }),
                  or a FV node name per arm (dict, {arm:fv_node}),
                  or None (NODE_NAMES==FV_NAMES).
      Note: FV_NAMES and NODE_NAMES are automatically guessed from the robot name.
    fv.fv
    fv.fv 'clear' [, ARM1 [, ARM2]]
      Stop to subscribe topics.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'is_active' [, ARM1 [, ARM2]]
      Check if FingerVision is working properly.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'pause' [, ARM1 [, ARM2]]
      Pause video processing.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'resume' [, ARM1 [, ARM2]]
      Resume video processing.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'clear_obj' [, ARM1 [, ARM2]]
      Clear detected object models.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'stop_detect_obj' [, ARM1 [, ARM2]]
      Stop detecting object.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'start_detect_obj' [, ARM1 [, ARM2]]
      Start detecting object.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'frame_skip', SKIP [, ARM1 [, ARM2]]
      Set frame-skip to SKIP.
      SKIP: Frames to be skipped. 0: No skip.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'take_snapshot', [PREFIX [, EXT [, ARM1 [, ARM2]]]]
      Take snapshots of current images.
      PREFIX: Snapshot prefix (default: ct.DataBaseDir()+'tmp/fv')
      EXT: Extension (default: '.png')
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
  '''

SRV_TABLE=[
  (std_srvs.srv.Empty, ('clear_obj','pause','resume','start_detect_obj','stop_detect_obj')),
  (fingervision_msgs.srv.SetInt32, ('set_frame_skip',)),
  (fingervision_msgs.srv.TakeSnapshot, ('take_snapshot',)),
  ]

'''
fv_names: FV camera names (dict, {LEFT/RIGHT:fv_l/fv_r}).
node_names: FV node names (dict, {LEFT/RIGHT:fv_l_node/fv_r_node}), or a single common node name.
    If node_names is None, fv_names is used as node_names.
'''
def GetFVSrvDict(fv_names, node_names=None):
  if node_names is None:  node_names= fv_names
  table= {
        'srv_separated':    not isinstance(node_names, str),
        fv_names[RIGHT]:    RIGHT,
        fv_names[LEFT]:     LEFT,
        }
  if table['srv_separated']:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      table[srv+'_r']= '/fingervision/{}/{}'.format(node_names[RIGHT],srv)
      table[srv+'_l']= '/fingervision/{}/{}'.format(node_names[LEFT],srv)
  else:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      table[srv]= '/fingervision/{}/{}'.format(node_names,srv)
  return table

#Return a table of FV topics and services for each (robot, arm).
def RobotToFV(robot, arm, no_exception=False):
  if robot.Is('Baxter'):
    if arm==RIGHT:  return GetFVSrvDict({RIGHT:'fv_pi01_r',LEFT:'fv_pi01_l'}, 'fv_pi01')
    elif arm==LEFT: return GetFVSrvDict({RIGHT:'fv_pi02_r',LEFT:'fv_pi02_l'}, 'fv_pi02')
  elif robot.Is('RobotiqNB'):
    return GetFVSrvDict({RIGHT:'fv_pi10_r',LEFT:'fv_pi10_l'})
  elif robot.Is('DxlGripper') or robot.Is('Mikata'):
    return GetFVSrvDict({RIGHT:'fv_pi11_r',LEFT:'fv_pi11_l'})
  elif not robot.Is('UR5eThG') and (robot.Is('URDxlG') or robot.Is('URThG') or robot.Is('Gen3') or robot.Is('RHP12RNGripper') or robot.Is('EZGripper')):
    return GetFVSrvDict({RIGHT:'fv_pi13_r',LEFT:'fv_pi13_l'})
  elif robot.Is('UR5eThG'):
    return GetFVSrvDict({RIGHT:'fv_pi15_r',LEFT:'fv_pi15_l'})
  elif not no_exception:
    raise Exception('fv.fv: No info in the RobotToFV for: {robot}=Arm-{arm}'.format(robot=robot.Name,arm=robot.ArmStr(arm)))
  return None

def Filter1Wrench(ct,arms,msg):
  for arm in arms:
    arm_S= ct.robot.ArmStrS(arm)
    table= ct.GetAttr(TMP,'fvconf'+arm_S)
    if msg.fv in table:
      l= ct.GetAttr(TMP,'fv'+arm_S)
      break
  side= table[msg.fv]
  l.posforce_array[side]= np.array(msg.posforce_array).reshape(len(msg.posforce_array)/5,5).tolist()
  l.force_array[side]= np.array(msg.force_array).reshape(len(msg.force_array)/6,6).tolist()
  l.dstate_array[side]= msg.dstate_array
  l.force[side]= msg.force
  l.dstate[side]= msg.dstate
  l.tm_last_topic[side]= msg.header.stamp

  if ct.callback.fv_wrench[ct.robot.ArmStrS(l.arm)][side] is not None:
    ct.callback.fv_wrench[ct.robot.ArmStrS(l.arm)][side](ct, l, side)

  #Broadcast the TF of FV.
  lw_xe= ct.GetAttr('wrist_'+LRToStrs(l.arm),'lx')
  gpos= ct.robot.GripperPos(l.arm)
  if gpos is not None:
    lw_xg= Transform(lw_xe,[0,(-0.5*gpos,+0.5*gpos)[side],0, 0,0,0,1])
    ct.br.sendTransform(lw_xg[0:3],lw_xg[3:],
        msg.header.stamp, msg.header.frame_id, ct.robot.EndLink(l.arm))

def Filter1ObjInfo(ct,arms,msg):
  for arm in arms:
    arm_S= ct.robot.ArmStrS(arm)
    table= ct.GetAttr(TMP,'fvconf'+arm_S)
    if msg.fv in table:
      l= ct.GetAttr(TMP,'fv'+arm_S)
      break
  side= table[msg.fv]
  l.obj_s[side]= msg.obj_s
  l.mv_s[side]= msg.mv_s
  l.obj_center[side]= msg.obj_center
  l.obj_orientation[side]= msg.obj_orientation
  l.obj_area[side]= msg.obj_area
  l.d_obj_center[side]= msg.d_obj_center
  l.d_obj_orientation[side]= msg.d_obj_orientation
  l.d_obj_area[side]= msg.d_obj_area
  l.obj_area_filtered[side]= msg.obj_area_filtered
  l.d_obj_center_filtered[side]= msg.d_obj_center_filtered
  l.d_obj_orientation_filtered[side]= msg.d_obj_orientation_filtered
  l.d_obj_area_filtered[side]= msg.d_obj_area_filtered
  l.tm_last_topic[2+side]= msg.header.stamp

  if ct.callback.fv_objinfo[ct.robot.ArmStrS(l.arm)][side] is not None:
    ct.callback.fv_objinfo[ct.robot.ArmStrS(l.arm)][side](ct, l, side)


def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  def read_arms(args):
    arms= []
    for a in args if args is not None and len(args)>0 else ['all']:
      if a in (LEFT,RIGHT):  arms.append(a)
      elif a=='all':  arms+= list(range(ct.robot.NumArms))
      else:  raise Exception('Too many arguments.')
    return set(arms)

  if command in ('on','setup'):
    arms= []
    fv_names,node_names= None,None
    no_wrench,no_objinfo= False,False
    for a in args:
      if a in (LEFT,RIGHT):  arms.append(a)
      elif a=='all':  arms+= list(range(ct.robot.NumArms))
      elif a=='no_wrench':  no_wrench= True
      elif a=='no_objinfo': no_objinfo= True
      elif fv_names is None:  fv_names= a
      elif node_names is None:  node_names= a
      else:  raise Exception('Too many arguments.')
    arms= set(arms)
    print '''Setup FV:
    arms: {}
    fv_names: {}
    node_names: {}
    no_wrench: {}
    no_objinfo: {}'''.format(arms,fv_names,node_names,no_wrench,no_objinfo)

    for arm in arms:
      arm_S= ct.robot.ArmStrS(arm)
      ct.SetAttr(TMP,'fv'+arm_S, TContainer())
      if 'fv_wrench' not in ct.callback:  ct.callback.fv_wrench= TContainer()
      ct.callback.fv_wrench[arm_S]= [None,None]  #Callback in Filter1Wrench
      if 'fv_objinfo' not in ct.callback:  ct.callback.fv_objinfo= TContainer()
      ct.callback.fv_objinfo[arm_S]= [None,None]  #Callback in Filter1ObjInfo
      l= ct.GetAttr(TMP,'fv'+arm_S)
      l.arm= arm
      l.fv= 'fv'+arm_S
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
      l.d_obj_center= [None,None]
      l.d_obj_orientation= [None,None]
      l.d_obj_area= [None,None]
      l.obj_area_filtered= [None,None]
      l.d_obj_center_filtered= [None,None]
      l.d_obj_orientation_filtered= [None,None]
      l.d_obj_area_filtered= [None,None]
      l.tm_last_topic= [None,None,None,None]  #ros time of last topic receiving. wrench-r,l, objinfo-r,l
      l.running= True
      l.no_wrench= no_wrench
      l.no_objinfo= no_objinfo

      if fv_names is None or arm_S not in fv_names or fv_names[arm_S] is None:
        table= RobotToFV(ct.robot, arm)
        print 'Found info for: {robot}=Arm-{arm}'.format(robot=ct.robot.Name,arm=ct.robot.ArmStr(arm))
      else:
        table= GetFVSrvDict(fv_names[arm_S],node_names[arm_S] if node_names is not None and arm_S in node_names else None)
        print 'Configured info with:',fv_names[arm_S],node_names[arm_S] if node_names is not None and arm_S in node_names else None
      ct.SetAttr(TMP,'fvconf'+arm_S, table)
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        for srvtype,srvs in SRV_TABLE:
          for srv in srvs:
            ct.AddSrvP(armstr+srv+'_l', table[srv+'_l'], srvtype, persistent=False, time_out=3.0)
            ct.AddSrvP(armstr+srv+'_r', table[srv+'_r'], srvtype, persistent=False, time_out=3.0)
      else:
        for srvtype,srvs in SRV_TABLE:
          for srv in srvs:
            ct.AddSrvP(armstr+srv, table[srv], srvtype, persistent=False, time_out=3.0)

      if table['srv_separated']:
        ct.srvp[armstr+'start_detect_obj_r'](std_srvs.srv.EmptyRequest())
        ct.srvp[armstr+'start_detect_obj_l'](std_srvs.srv.EmptyRequest())
      else:
        ct.srvp[armstr+'start_detect_obj'](std_srvs.srv.EmptyRequest())

    if no_wrench:
      ct.AddSub('fv_filter1_wrench', '/fingervision/fv_filter1_wrench', fingervision_msgs.msg.Filter1Wrench, lambda msg,arms=arms:Filter1Wrench(ct,arms,msg))
    else:
      ct.AddSubW('fv_filter1_wrench', '/fingervision/fv_filter1_wrench', fingervision_msgs.msg.Filter1Wrench, lambda msg,arms=arms:Filter1Wrench(ct,arms,msg), time_out=3.0)
    if no_objinfo:
      ct.AddSub('fv_filter1_objinfo', '/fingervision/fv_filter1_objinfo', fingervision_msgs.msg.Filter1ObjInfo, lambda msg,arms=arms:Filter1ObjInfo(ct,arms,msg))
    else:
      ct.AddSubW('fv_filter1_objinfo', '/fingervision/fv_filter1_objinfo', fingervision_msgs.msg.Filter1ObjInfo, lambda msg,arms=arms:Filter1ObjInfo(ct,arms,msg), time_out=3.0)

  elif command=='clear':
    arms= read_arms(args)
    for arm in arms:
      arm_S= ct.robot.ArmStrS(arm)
      print 'Stopping','fv'+arm_S
      if ct.HasAttr(TMP,'fv'+arm_S):
        ct.GetAttr(TMP,'fv'+arm_S).running= False
      if 'fv_wrench' in ct.callback:  ct.callback.fv_wrench[arm_S]= [None,None]
      if 'fv_objinfo' in ct.callback:  ct.callback.fv_objinfo[arm_S]= [None,None]

      table= ct.GetAttrOr(None,TMP,'fvconf'+arm_S)
      if table is None:  continue
      armstr= ct.robot.ArmStr(arm)+'_'
      for srv in ('fv_filter1_wrench','fv_filter1_objinfo'):
        ct.DelSub(srv)
      if table['srv_separated']:
        for srv in sum((srvs for _,srvs in SRV_TABLE),()):
          ct.DelSrvP(armstr+srv+'_r')
          ct.DelSrvP(armstr+srv+'_l')
      else:
        for srv in sum((srvs for _,srvs in SRV_TABLE),()):
          ct.DelSrvP(armstr+srv)

  elif command=='is_active':
    arms= read_arms(args)
    is_active= [False]*ct.robot.NumArms
    for arm in arms:
      arm_S= ct.robot.ArmStrS(arm)
      if not ct.HasAttr(TMP,'fv'+arm_S):
        is_active[arm]= False
      else:
        l= ct.GetAttr(TMP,'fv'+arm_S)
        tm_last_topic= [] if l.no_wrench else l.tm_last_topic[:2]
        tm_last_topic+= [] if l.no_objinfo else l.tm_last_topic[2:]
        is_active[arm]= None not in tm_last_topic and len(tm_last_topic)>0 \
          and (rospy.Time.now()-min(tm_last_topic)).to_sec()<0.2
    return is_active

  elif command in SRV_TABLE[0][1]:
    arms= read_arms(args)
    for arm in arms:
      table= ct.GetAttr(TMP,'fvconf'+ct.robot.ArmStrS(arm))
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        ct.srvp[armstr+command+'_r'](std_srvs.srv.EmptyRequest())
        ct.srvp[armstr+command+'_l'](std_srvs.srv.EmptyRequest())
      else:
        ct.srvp[armstr+command](std_srvs.srv.EmptyRequest())

  elif command=='frame_skip':
    skip= args[0] if len(args)>0 else None
    arms= read_arms(args[1:])
    set_frame_skip_req= fingervision_msgs.srv.SetInt32Request()
    set_frame_skip_req.data= skip
    for arm in arms:
      table= ct.GetAttr(TMP,'fvconf'+ct.robot.ArmStrS(arm))
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        ct.srvp[armstr+'set_frame_skip_r'](set_frame_skip_req)
        ct.srvp[armstr+'set_frame_skip_l'](set_frame_skip_req)
      else:
        ct.srvp[armstr+'set_frame_skip'](set_frame_skip_req)

  elif command=='take_snapshot':
    prefix= args[0] if len(args)>0 else ct.DataBaseDir()+'tmp/fv'
    extension= args[1] if len(args)>1 else '.png'
    arms= read_arms(args[2:])
    take_snapshot_req= fingervision_msgs.srv.TakeSnapshotRequest()
    take_snapshot_req.prefix= prefix
    take_snapshot_req.ext= extension
    files= {}
    for arm in arms:
      table= ct.GetAttr(TMP,'fvconf'+ct.robot.ArmStrS(arm))
      armstr= ct.robot.ArmStr(arm)+'_'
      files[arm]= []
      if table['srv_separated']:
        files[arm]+= ct.srvp[armstr+'take_snapshot_r'](take_snapshot_req).files
        files[arm]+= ct.srvp[armstr+'take_snapshot_l'](take_snapshot_req).files
      else:
        files[arm]= ct.srvp[armstr+'take_snapshot'](take_snapshot_req).files
    return files


#!/usr/bin/python
from core_tool import *
import std_srvs.srv
roslib.load_manifest('fingervision_msgs')
import fingervision_msgs.msg
import fingervision_msgs.srv
def Help():
  return '''FingerVision utility.
  Usage:
    fv.fv 'on' [, ARM1 [, ARM2]]
    fv.fv 'setup' [, ARM1 [, ARM2]]
      Start to subscribe topics, setup services.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv
    fv.fv 'clear' [, ARM1 [, ARM2]]
      Stop to subscribe topics.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'is_active' [, ARM1 [, ARM2]]
      Check if FingerVision is working properly.
      ARM*: RIGHT, LEFT, or 'all' (all arms). Default: 'all'
    fv.fv 'frame_skip', SKIP [, ARM1 [, ARM2]]
      Set frame-skip to SKIP.
      SKIP: Frames to be skipped. 0: No skip.
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
  '''

#Return a table of FV topics and services for each (robot, arm).
def RobotToFV(robot, arm, no_exception=False):
  if robot.Is('Baxter'):
    if arm==RIGHT:
      return {
        'srv_separated':    False,
        'fv_pi01_r':        RIGHT,
        'fv_pi01_l':        LEFT,
        'clear_obj':        '/fingervision/fv_pi01/clear_obj',
        'set_frame_skip':   '/fingervision/fv_pi01/set_frame_skip',
        'start_detect_obj': '/fingervision/fv_pi01/start_detect_obj',
        'stop_detect_obj':  '/fingervision/fv_pi01/stop_detect_obj',
        }
    elif arm==LEFT:
      return {
        'srv_separated':    False,
        'fv_pi02_r':        RIGHT,
        'fv_pi02_l':        LEFT,
        'clear_obj':        '/fingervision/fv_pi02/clear_obj',
        'set_frame_skip':   '/fingervision/fv_pi02/set_frame_skip',
        'start_detect_obj': '/fingervision/fv_pi02/start_detect_obj',
        'stop_detect_obj':  '/fingervision/fv_pi02/stop_detect_obj',
        }
  elif robot.Is('RobotiqNB'):
    return {
      'srv_separated':      True,
      'fv_pi10_r':          RIGHT,
      'fv_pi10_l':          LEFT,
      'clear_obj_r':        '/fingervision/fv_pi10_r/clear_obj',
      'clear_obj_l':        '/fingervision/fv_pi10_l/clear_obj',
      'set_frame_skip_r':   '/fingervision/fv_pi10_r/set_frame_skip',
      'set_frame_skip_l':   '/fingervision/fv_pi10_l/set_frame_skip',
      'start_detect_obj_r': '/fingervision/fv_pi10_r/start_detect_obj',
      'start_detect_obj_l': '/fingervision/fv_pi10_l/start_detect_obj',
      'stop_detect_obj_r':  '/fingervision/fv_pi10_r/stop_detect_obj',
      'stop_detect_obj_l':  '/fingervision/fv_pi10_l/stop_detect_obj',
      }
  elif robot.Is('DxlGripper') or robot.Is('Mikata'):
    return {
      'srv_separated':      True,
      'fv_pi11_r':          RIGHT,
      'fv_pi11_l':          LEFT,
      'clear_obj_r':        '/fingervision/fv_pi11_r/clear_obj',
      'clear_obj_l':        '/fingervision/fv_pi11_l/clear_obj',
      'set_frame_skip_r':   '/fingervision/fv_pi11_r/set_frame_skip',
      'set_frame_skip_l':   '/fingervision/fv_pi11_l/set_frame_skip',
      'start_detect_obj_r': '/fingervision/fv_pi11_r/start_detect_obj',
      'start_detect_obj_l': '/fingervision/fv_pi11_l/start_detect_obj',
      'stop_detect_obj_r':  '/fingervision/fv_pi11_r/stop_detect_obj',
      'stop_detect_obj_l':  '/fingervision/fv_pi11_l/stop_detect_obj',
      }
  elif not robot.Is('UR5eThG') and (robot.Is('URDxlG') or robot.Is('URThG') or robot.Is('RHP12RNGripper')):
    return {
      'srv_separated':      True,
      'fv_pi13_r':          RIGHT,
      'fv_pi13_l':          LEFT,
      'clear_obj_r':        '/fingervision/fv_pi13_r/clear_obj',
      'clear_obj_l':        '/fingervision/fv_pi13_l/clear_obj',
      'set_frame_skip_r':   '/fingervision/fv_pi13_r/set_frame_skip',
      'set_frame_skip_l':   '/fingervision/fv_pi13_l/set_frame_skip',
      'start_detect_obj_r': '/fingervision/fv_pi13_r/start_detect_obj',
      'start_detect_obj_l': '/fingervision/fv_pi13_l/start_detect_obj',
      'stop_detect_obj_r':  '/fingervision/fv_pi13_r/stop_detect_obj',
      'stop_detect_obj_l':  '/fingervision/fv_pi13_l/stop_detect_obj',
      }
  elif robot.Is('UR5eThG'):
    return {
      'srv_separated':      True,
      'fv_pi15_r':          RIGHT,
      'fv_pi15_l':          LEFT,
      'clear_obj_r':        '/fingervision/fv_pi15_r/clear_obj',
      'clear_obj_l':        '/fingervision/fv_pi15_l/clear_obj',
      'set_frame_skip_r':   '/fingervision/fv_pi15_r/set_frame_skip',
      'set_frame_skip_l':   '/fingervision/fv_pi15_l/set_frame_skip',
      'start_detect_obj_r': '/fingervision/fv_pi15_r/start_detect_obj',
      'start_detect_obj_l': '/fingervision/fv_pi15_l/start_detect_obj',
      'stop_detect_obj_r':  '/fingervision/fv_pi15_r/stop_detect_obj',
      'stop_detect_obj_l':  '/fingervision/fv_pi15_l/stop_detect_obj',
      }
  elif not no_exception:
    raise Exception('fv.fv: No info in the RobotToFV for: {robot}=Arm-{arm}'.format(robot=robot.Name,arm=robot.ArmStr(arm)))
  return None

def Filter1Wrench(ct,l,msg):
  table= RobotToFV(ct.robot, l.arm)
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
  lw_xg= Transform(lw_xe,[0,(-0.5*gpos,+0.5*gpos)[side],0, 0,0,0,1])
  ct.br.sendTransform(lw_xg[0:3],lw_xg[3:],
      rospy.Time.now(), msg.header.frame_id, ct.robot.EndLink(l.arm))

def Filter1ObjInfo(ct,l,msg):
  table= RobotToFV(ct.robot, l.arm)
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

  if command in ('on','setup'):
    if len(args)==0:  args= ['all']
    arms= set(sum([range(ct.robot.NumArms) if a=='all' else [a] for a in args],[]))
    if ct.robot.NumArms==1:
      arms= set(0 if a>0 else a for a in arms)

    for arm in arms:
      arm_S= ct.robot.ArmStrS(arm)
      ct.SetAttr(TMP,'fv'+arm_S, TContainer(debug=True))
      if 'fv_wrench' not in ct.callback:  ct.callback.fv_wrench= TContainer(debug=True)
      ct.callback.fv_wrench[arm_S]= [None,None]  #Callback in Filter1Wrench
      if 'fv_objinfo' not in ct.callback:  ct.callback.fv_objinfo= TContainer(debug=True)
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

      table= RobotToFV(ct.robot, arm)
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        x='clear_obj_r'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
        x='clear_obj_l'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
        x='set_frame_skip_r'; ct.AddSrvP(armstr+x, table[x], fingervision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
        x='set_frame_skip_l'; ct.AddSrvP(armstr+x, table[x], fingervision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
        x='start_detect_obj_r'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
        x='start_detect_obj_l'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
        x='stop_detect_obj_r'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
        x='stop_detect_obj_l'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
      else:
        x='clear_obj'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
        x='set_frame_skip'; ct.AddSrvP(armstr+x, table[x], fingervision_msgs.srv.SetInt32, persistent=False, time_out=3.0)
        x='start_detect_obj'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)
        x='stop_detect_obj'; ct.AddSrvP(armstr+x, table[x], std_srvs.srv.Empty, persistent=False, time_out=3.0)

      if table['srv_separated']:
        ct.srvp[armstr+'start_detect_obj_r'](std_srvs.srv.EmptyRequest())
        ct.srvp[armstr+'start_detect_obj_l'](std_srvs.srv.EmptyRequest())
        #ct.srvp[armstr+'clear_obj_r'](std_srvs.srv.EmptyRequest())
        #ct.srvp[armstr+'clear_obj_l'](std_srvs.srv.EmptyRequest())
      else:
        ct.srvp[armstr+'start_detect_obj'](std_srvs.srv.EmptyRequest())
        #ct.srvp[armstr+'clear_obj'](std_srvs.srv.EmptyRequest())

    ct.AddSub('fv_filter1_wrench', '/fingervision/fv_filter1_wrench', fingervision_msgs.msg.Filter1Wrench, lambda msg,l=l:Filter1Wrench(ct,l,msg))
    ct.AddSub('fv_filter1_objinfo', '/fingervision/fv_filter1_objinfo', fingervision_msgs.msg.Filter1ObjInfo, lambda msg,l=l:Filter1ObjInfo(ct,l,msg))

  elif command=='clear':
    if len(args)==0:  args= ['all']
    arms= set(sum([range(ct.robot.NumArms) if a=='all' else [a] for a in args],[]))
    for arm in arms:
      arm_S= ct.robot.ArmStrS(arm)
      print 'Stopping','fv'+arm_S
      if ct.HasAttr(TMP,'fv'+arm_S):
        ct.GetAttr(TMP,'fv'+arm_S).running= False
      if 'fv_wrench' in ct.callback:  ct.callback.fv_wrench[arm_S]= [None,None]
      if 'fv_objinfo' in ct.callback:  ct.callback.fv_objinfo[arm_S]= [None,None]

      table= RobotToFV(ct.robot, arm, no_exception=True)
      if table is None:  continue
      armstr= ct.robot.ArmStr(arm)+'_'
      for x in ('fv_filter1_wrench','fv_filter1_objinfo'):
        ct.DelSub(x)
      if table['srv_separated']:
        for x in ('clear_obj_r','clear_obj_l','set_frame_skip_r','set_frame_skip_l',
                  'start_detect_obj_r','start_detect_obj_l','stop_detect_obj_r','stop_detect_obj_l'):
          ct.DelSrvP(armstr+x)
      else:
        for x in ('clear_obj','set_frame_skip','start_detect_obj','stop_detect_obj'):
          ct.DelSrvP(armstr+x)

  elif command=='is_active':
    if len(args)==0:  args= ['all']
    arms= set(sum([range(ct.robot.NumArms) if a=='all' else [a] for a in args],[]))
    is_active= [False]*ct.robot.NumArms
    for arm in arms:
      arm_S= ct.robot.ArmStrS(arm)
      if not ct.HasAttr(TMP,'fv'+arm_S):
        is_active[arm]= False
      else:
        is_active[arm]= None not in ct.GetAttr(TMP,'fv'+arm_S).tm_last_topic \
          and (rospy.Time.now()-min(ct.GetAttr(TMP,'fv'+arm_S).tm_last_topic)).to_sec()<0.2
    return is_active

  elif command=='frame_skip':
    skip= args[0] if len(args)>0 else None
    if len(args[1:])==0:  args= [None,'all']
    arms= set(sum([range(ct.robot.NumArms) if a=='all' else [a] for a in args[1:]],[]))
    set_frame_skip_req= fingervision_msgs.srv.SetInt32Request()
    set_frame_skip_req.data= skip
    for arm in arms:
      table= RobotToFV(ct.robot, arm)
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        ct.srvp[armstr+'set_frame_skip_r'](set_frame_skip_req)
        ct.srvp[armstr+'set_frame_skip_l'](set_frame_skip_req)
      else:
        ct.srvp[armstr+'set_frame_skip'](set_frame_skip_req)

  elif command=='clear_obj':
    if len(args)==0:  args= ['all']
    arms= set(sum([range(ct.robot.NumArms) if a=='all' else [a] for a in args],[]))
    for arm in arms:
      table= RobotToFV(ct.robot, arm)
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        ct.srvp[armstr+'clear_obj_r'](std_srvs.srv.EmptyRequest())
        ct.srvp[armstr+'clear_obj_l'](std_srvs.srv.EmptyRequest())
      else:
        ct.srvp[armstr+'clear_obj'](std_srvs.srv.EmptyRequest())

  elif command=='stop_detect_obj':
    if len(args)==0:  args= ['all']
    arms= set(sum([range(ct.robot.NumArms) if a=='all' else [a] for a in args],[]))
    for arm in arms:
      table= RobotToFV(ct.robot, arm)
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        ct.srvp[armstr+'stop_detect_obj_r'](std_srvs.srv.EmptyRequest())
        ct.srvp[armstr+'stop_detect_obj_l'](std_srvs.srv.EmptyRequest())
      else:
        ct.srvp[armstr+'stop_detect_obj'](std_srvs.srv.EmptyRequest())

  elif command=='start_detect_obj':
    if len(args)==0:  args= ['all']
    arms= set(sum([range(ct.robot.NumArms) if a=='all' else [a] for a in args],[]))
    for arm in arms:
      table= RobotToFV(ct.robot, arm)
      armstr= ct.robot.ArmStr(arm)+'_'
      if table['srv_separated']:
        ct.srvp[armstr+'start_detect_obj_r'](std_srvs.srv.EmptyRequest())
        ct.srvp[armstr+'start_detect_obj_l'](std_srvs.srv.EmptyRequest())
      else:
        ct.srvp[armstr+'start_detect_obj'](std_srvs.srv.EmptyRequest())


#!/usr/bin/python
from core_tool import *
def Help():
  return '''Handle the virtual scene for planning.
  Usage:
    scene
    scene 'clear'
      Clear the virtual scene to default
    scene 'make' [, IGN_OBJS [, BB_MARGIN [, SITUATION]]]
      Construct the virtual scene with putting objects described in attribute [TMP]['scene']
      IGN_OBJS: A list of objects to be ignored (default: [])
      BB_MARGIN: Margin ratio of a bounding box (default: 1.1)
      SITUATION: Situation dict to return the object information (default: N/A)
    scene 'isvalidq', ARM [, IGN_OBJS [, Q [, VISUALIZE]]]
      Check the validity of joint angles
      ARM: Hand ID
      IGN_OBJS: A list of objects to be ignored (default: [])
      Q: Joint angles (default: ct.robot.Q(ARM))
      VISUALIZE: Visualize or not (default: False)
      Return: validity, result of scene checker
        Note: IGN_OBJS will be included in the result of scene checker
    scene 'isvalidx', ARM, X [, IGN_OBJS [, Q_INIT [, VISUALIZE]]]
      Check the validity of Cartesian pose in the wrist frame
      ARM: Hand ID
      X: Cartesian pose
      IGN_OBJS: A list of objects to be ignored (default: [])
      Q_INIT: Initial joint angles to solve for X (default: ct.robot.Q(ARM))
      VISUALIZE: Visualize or not (default: False)
      Return: validity, result of IK checker, result of scene checker
        Note: IGN_OBJS will be included in the result of scene checker
    scene 'grab', ARM, OBJ
      Virtually attach OBJ to the hand of ARM.
      ARM: Hand ID
      OBJ: Object
    scene 'release', OBJ
      Virtually detach the OBJ from the hand.
      OBJ: Object
  '''
def Run(ct,*args):
  res= True
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  displavel= ct.Run('displevel','shownum')

  if command=='clear':
    if displavel>0:  CPrint(3, 'Clearing scene')
    with ct.robot.sensor_locker:
      ct.state_validity_checker.RemoveFromScene()
    if 'scene' in ct.viz:
      del ct.viz['scene']
  elif command=='make':
    ign_objs= set(args[0]) if len(args)>0 else set()
    bb_margin= args[1] if len(args)>1 else 1.1
    situation= args[2] if len(args)>2 else {}

    #Load scene:
    objs= set(ct.GetAttrOr([], TMP,'scene'))
    cmn= objs&ign_objs  #Common in objs and ign_objs
    objs-= cmn
    ign_objs-= cmn

    if displavel>0:  CPrint(3, 'Making scene:',objs,'ignoring:',ign_objs,'margin:',bb_margin)

    ct.viz.scene= TSimpleVisualizer(name_space='visualizer_scene')
    ct.viz.scene.viz_frame= ct.robot.BaseFrame

    ct.state_validity_checker.InitToMakeScene()

    with ct.robot.sensor_locker:
      x_w_lr= [ct.robot.FK(arm=arm) for arm in range(ct.robot.NumArms)]
      #Add virtual collision models
      virtual_components= ['wrist_r']
      if ct.robot.Is('Baxter'): virtual_components= ['wrist_r','wrist_l','wl_m100','wr_stereo']

      for wobj in virtual_components:
        if ct.HasAttr(wobj):
          bound_box= ct.GetAttr(wobj, 'bound_box')
          arm= StrToLR(ct.GetAttr(wobj, 'kinematics_chain','parent_arm'))
          lw_xe= ct.GetAttr(wobj,'lx')
          #xe= Vec(Transform(x_w_lr[arm],lw_xe))
          lx_bb= bound_box['center']
          dim_bb= Vec(bound_box['dim'])*bb_margin
          x_bb= Transform(x_w_lr[arm], lx_bb) #Transform(xe, lx_bb)  #A
          #lwx_bb= TransformLeftInv(x_w_lr[arm], x_bb)  #A
          lwx_bb= lx_bb  #A
          #A: Modified on 2018-10-16 for fixing the bug, tested with Motoman.
          ct.state_validity_checker.AddBoxToRobotHand(lwx_bb,dim_bb,arm=arm,name=wobj)
          ct.viz.scene.AddCube(x_bb, dim_bb, rgb=ct.viz.scene.ICol(1), alpha=0.3)
          if displavel>0:  CPrint(3, '  attaching %r to %s-hand'%(wobj,ct.robot.ArmStr(arm)))
      #Add virtual collision models
      for obj in objs:
        ct.Run('adv.infer_x', obj)
        if not ct.HasAttr(obj,'x'):
          CPrint(4,'Error: cannot add %r to scene' % obj)
          res= False
          continue
        x_o= ct.GetAttr(obj,'x')
        ct.Run('adv.infer_bb', obj)
        if not ct.HasAttr(obj, 'bound_box'):
          CPrint(4,'Error: cannot get the bound box of %r' % obj)
          res= False
          continue
        bound_box= ct.GetAttr(obj, 'bound_box')
        lo_o_center= bound_box['center']
        o_dim= Vec(bound_box['dim'])*bb_margin
        o_center= Transform(x_o, lo_o_center)
        if ct.HasAttr(obj,'grabbed'):
          grab_arm= ct.GetAttr(obj,'grabbed','grabber_handid')
          lw_o_center= TransformLeftInv(x_w_lr[grab_arm],o_center)
          ct.state_validity_checker.AddBoxToRobotHand(lw_o_center,o_dim,arm=grab_arm,name=obj)
          InsertDict(situation, {'objs_attr':{obj:{'x':x_o,'grabbed':{'grabber_handid':grab_arm}}}})
          if displavel>0:  CPrint(3, '  attaching %r to %s-hand'%(obj,LRToStr(grab_arm)))
        else:
          ct.state_validity_checker.AddBoxToScene(o_center,o_dim,name=obj)
          InsertDict(situation, {'objs_attr':{obj:{'x':x_o}}})
          if displavel>0:  CPrint(3, '  adding %r to scene'%(obj))
        for obj2 in ign_objs:
          if displavel>0:  CPrint(3, '  ignoring collision between %r and %r'%(obj,obj2))
          ct.state_validity_checker.IgnoreCollision(obj,obj2)
        if ct.robot.Is('PR2'):
          #Ignore collision of obj with 'collision_map'
          if displavel>0:  CPrint(3, '  ignoring collision between %r and %r'%(obj,'collision_map'))
          ct.state_validity_checker.IgnoreCollision(obj,'collision_map')
        ct.viz.scene.AddCube(o_center, o_dim, rgb=ct.viz.scene.ICol(1), alpha=0.3)
      for obj2 in ign_objs:
        if displavel>0:  CPrint(3, '  ignoring collision between %r and %r'%('all',obj2))
        ct.state_validity_checker.IgnoreCollision('all',obj2)
      #Ignoring list
      ign_combinations= ()
      if ct.robot.Is('PR2'):
        ign_combinations= (('all','collision_map'),
                ('.robot','collision_map'),
                ('.l_gripper','l_gripper_sensor_mount_link'))
      elif ct.robot.Is('Baxter'):
        ign_combinations= (('display','.head'),
                ('right_gripper_base','.r_gripper'),
                ('right_upper_elbow_visual','torso'),
                ('left_upper_elbow_visual','torso'))
      elif ct.robot.Is('Motoman'):
        ign_combinations= (('base_link', 'link_s'),
                           ('link_s', 'link_l'),('link_l', 'link_e'),('link_e', 'link_u'),
                           ('link_u', 'link_r'),('link_r', 'link_b'),('link_b', 'link_t'),
                           ('link_b', 'wrist_r'),('link_t', 'wrist_r'))
      elif ct.robot.Is('Mikata'):
        ign_combinations= (('base_link','link_2'),('link_2','link_3'),('link_3','link_4'),('link_4','link_5'),
                           ('link_5','right_gripper'),('link_5','left_gripper'),('right_gripper','left_gripper'))
      elif ct.robot.Is('UR'):
        ign_combinations= (('base_link','shoulder_link'),('shoulder_link','upper_arm_link'),
                           ('upper_arm_link','forearm_link'),('forearm_link','wrist_1_link'),
                           ('wrist_1_link','wrist_2_link'),('wrist_2_link','wrist_3_link'),
                           ('wrist_3_link','ee_link'))
      for i1,i2 in ign_combinations:
        if displavel>0:  CPrint(3, '  ignoring collision between %r and %r'%(i1,i2))
        ct.state_validity_checker.IgnoreCollision(i1,i2)

      ct.state_validity_checker.SendToServer()
      #print xxx.planning_scene.link_padding

  elif command=='isvalidq':
    arm= args[0]
    ign_objs= set(args[1]) if len(args)>1 else set()
    q= args[2] if len(args)>2 else ct.robot.Q(arm)
    visualization= args[3] if len(args)>3 else False

    with ct.robot.sensor_locker:
      res= ct.state_validity_checker.IsValidState(q, arm=arm)
    if visualization:  VisualizeContacts(res.contacts)
    if isinstance(ct.state_validity_checker, TStateValidityCheckerMI):
      if res.valid:
        return True, res
      else:
        col_objs= GetCollidingObjects(res.contacts, ign_objs)
        return len(col_objs)==0, res
    else:
      raise Exception('Unknown ct.state_validity_checker type:',type(ct.state_validity_checker))

  elif command=='isvalidx':
    arm= args[0]
    x= args[1]
    ign_objs= set(args[2]) if len(args)>2 else set()
    q_init= args[3] if len(args)>3 else ct.robot.Q(arm)
    visualization= args[4] if len(args)>4 else False
    #print '###DEBUG### \'isvalidx\'',','.join(map(str,args))

    q1,res1= ct.robot.IK(x, start_angles=q_init, arm=arm, with_st=True)
    if q1 is None:
      #print '###DEBUG### resD:',False, res1, None
      return False, res1, None
    with ct.robot.sensor_locker:
      res2= ct.state_validity_checker.IsValidState(q1, arm=arm)
    if visualization:  VisualizeContacts(res2.contacts)
    if isinstance(ct.state_validity_checker, TStateValidityCheckerMI):
      if res2.valid:
        return True, res1, res2
      else:
        col_objs= GetCollidingObjects(res2.contacts, ign_objs)
        return len(col_objs)==0, res1, res2

  elif command=='grab':
    arm= args[0]
    obj= args[1]
    if ct.HasAttr(obj,'grabbed'):
      print 'Error: already grabbed: ',obj
      return False
    x_o= ct.GetAttr(obj,'x')
    arms= ct.robot.ArmStrs(arm)
    grab_attr= {}
    grab_attr['grabber']= 'gripper_'+arms
    grab_attr['grabber_wrist']= 'wrist_'+LRToStrs(arm)
    grab_attr['grabber_hand']= arms
    grab_attr['grabber_handid']= arm
    grab_attr['joint_angles']= ct.robot.Q(arm)
    grab_attr['grabbed_x']= x_o
    grab_attr['l_x_grab']= TransformLeftInv(ct.robot.FK(x_ext=ct.GetAttr('wrist_'+LRToStrs(arm),'lx'),arm=arm),x_o)
    ct.AddDictAttr(obj,'grabbed',  grab_attr)
    return True

  elif command=='release':
    obj= args[0]
    if not ct.HasAttr(obj,'grabbed'):
      print 'Error: not grabbed: ',obj
      return False
    arm= ct.GetAttr(obj,'grabbed','grabber_handid')
    ct.DelAttr(obj,'grabbed')
    return True

  else:
    raise Exception('Invalid command: %r'%command)

  return res

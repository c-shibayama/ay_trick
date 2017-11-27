#!/usr/bin/python
from core_tool import *
def Help():
  return '''Plan a collision free trajectory.
  Usage: adv.plan_traj SITUATION [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)
  '''
def Run(ct,*args):
  situation= args[0]
  inferred_keys= args[1] if len(args)>1 else []
  verbose= args[2] if len(args)>2 else True

  if 'obj' in situation:
    CPrint(4,'situation[obj] for infer_traj is DEPRECATED.')
    raise
  ign_objs= GetOrSet(situation, 'ign_objs', lambda:[])
  arm= situation['handid']
  lw_x_ext= situation['l_x_ext']
  x_trg= situation['x_target']
  q_curr= GetOrSet(situation, 'q_curr', lambda:ct.robot.Q(arm))
  dt= GetOrSet(situation, 'dt', lambda:3.0)
  inum= GetOrSet(situation, 'N', lambda:20)
  bb_margin= GetOrSet(situation, 'bb_margin', lambda:1.1)

  t_traj= TimeTraj(dt,inum)

  viz= TSimpleVisualizer(rospy.Duration(10.0), name_space='visualizer_traj')
  viz.viz_frame= ct.robot.BaseFrame


  def SetupScene():
    ct.Run('scene', 'make',ign_objs,bb_margin,situation)

  def CleanScene():
    ct.Run('scene', 'clear')


  inferred_keys.append([TMP,'key_points'])
  #NOTE: 'q_traj', 't_traj', and 'x_traj' are also inferred, but it is not added to inferred_keys
  # in order to avoid to be stored in data_base


  #Evaluate a trajectory.
  #key_points: spline key points [[ct,x,y,z],...]
  #x_curr, x_trg, N: same as others
  #ct: TCoreTool instance
  #x_to_q_traj: function to transform a x trajectory to joint angle trajectory
  #to_ros_traj: function to convert q_traj to a ROS trajectory
  def EvalTraj(key_points, x_curr, x_trg, q_curr, N, ct, x_to_q_traj, to_ros_traj, verbose=False, return_traj=False):
    score= 1.0

    splines= [TCubicHermiteSpline() for d in range(len(key_points[0])-1)]
    for d in range(len(splines)):
      data_d= [[p[0],p[d+1]] for p in key_points]
      splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)

    p_traj=[]
    for n in range(N):
      time= float(n+1)/float(N)*(key_points[-1][0]-key_points[0][0]) + key_points[0][0]
      p= [splines[d].Evaluate(time) for d in range(len(splines))]
      p_traj.append(p)

    qo_traj= QInterpolation(x_curr[3:],x_trg[3:],N)
    x_traj= [list(p_traj[n])+list(qo_traj[n]) for n in range(N)]

    #Transform the trajectory to the wrist space:
    q_traj= x_to_q_traj(x_traj)
    if q_traj is None:  return None

    #TEST:Velocity consistency check:
    #NOTE:IKTrajCheck constrains too much, which seems to be inaccurate.
    #err= IKTrajCheck(x_traj, q_traj=[jp.positions for jp in q_traj.trajectory.points])
    #if err is None:  return None
    #score-= 100.0*err  #FIXME: using err as the penalty is good or bad?

    if return_traj:  return q_traj, x_traj

    q_traj_ros= to_ros_traj(q_traj)
    #For TStateValidityChecker:
    #with ct.robot.sensor_locker:
      #res= ct.state_validity_checker.IsValidTrajectory(q_traj_ros)
      #if verbose:  print 'res.error_code.val=',res.error_code.val
      #if verbose:  print 'res.trajectory_error_codes=',res.trajectory_error_codes
      #if res.error_code.val!=res.error_code.SUCCESS:
        ##Ignore goal collision:
        ##if np.array([e.val==res.error_code.SUCCESS for e in res.trajectory_error_codes[:-1]]).all() \
            ##and res.trajectory_error_codes[-1].val==res.error_code.COLLISION_CONSTRAINTS_VIOLATED:
          ##pass
        ##else:
          ##return None
        #return None
    #For TStateValidityCheckerMI:
    #with ct.robot.sensor_locker:
    res= ct.state_validity_checker.IsValidTrajectory(q_traj_ros)
    if not res.valid:
      if verbose: print 'IsValidTrajectory=',res
      return None

    dist= 0.0
    for n in range(len(key_points)-1):
      dist+= la.norm(Vec(key_points[n+1][1:])-Vec(key_points[n][1:]))
    dist/= la.norm(Vec(key_points[-1][1:])-Vec(key_points[0][1:]))
    score-= (dist-1.0)

    return score

  #Expand trajectory parameters.
  #parameters[0]: angle of the plane where the trajectory is planned
  #parameters[1,2]: distance and angle of 1st via-point centered at p0
  #parameters[3,4]: distance and angle of 2nd via-point centered at pf
  #note: distance (parameters[1,3]) are normalized by norm(pf-p0)
  #return: spline key points [ct,x,y,z]*4
  def ExpandTrajParameters(parameters,p0,pf):
    tf= 1.0
    p0= Vec(p0)
    pf= Vec(pf)
    if la.norm(pf-p0)<1.0e-10:
      raise Exception('infer_sub.infer_traj: Current and target points are too close:',p0,pf)
    ex= pf-p0
    ex= ex/la.norm(ex)
    ez= GetOrthogonalAxisOf(ex,preferable=[0.0,0.0,1.0],fault=[1.0,0.0,0.0])
    ey= np.cross(ez,ex)
    #print 'ex=',ex
    #print 'ey=',ey
    #print 'ez=',ez

    angle= parameters[0]
    norm0f= 0.5*la.norm(pf-p0)
    diff= lambda q1,q2: norm0f * (q1*ex*math.cos(q2) + q1*(math.sin(angle)*ey+math.cos(angle)*ez)*math.sin(q2))
    p= [[]]*2
    p[0]= p0 + diff(parameters[1],parameters[2])
    p[1]= pf + diff(parameters[3],math.pi-parameters[4])
    dt= np.array([la.norm(p0-p[0]),la.norm(p[0]-p[1]),la.norm(p[1]-pf)])
    dt= dt*(tf/sum(dt))
    data= []
    data.append([0.0]        +p0.tolist())
    data.append([dt[0]]      +p[0].tolist())
    data.append([dt[0]+dt[1]]+p[1].tolist())
    data.append([tf]         +pf.tolist())
    return data

  #def DistOfTrajExs(parameters, x_curr, x_trg, q_curr, N, ct, x_to_q_traj, to_ros_traj,  situation):
    #if situation['infer_info']['type']!='ver1':  return None
    #dists= []
    #dists.append(Norm(Vec(parameters) - Vec(situation['infer_info']['param'])))
    #d_x_trg= DiffX(x_trg, situation['x_target'])
    #dists.append(Norm(d_x_trg[:3]))
    #dists.append(Norm(d_x_trg[3:]))
    #dists.append(max(map(abs,map(AngleMod1,Vec(q_curr)-Vec(situation['q_curr'])))))
    ##print dists
    #dist= max(dists)
    #return dist

  #x_curr, x_trg, N: same as others
  #ct: TCoreTool instance
  #x_to_q_traj: function to transform a x trajectory to joint angle trajectory
  #to_ros_traj: function to convert q_traj to a ROS trajectory
  def EvalTrajForFMin(parameters, x_curr, x_trg, q_curr, N, ct, x_to_q_traj, to_ros_traj, f_none=100):
    key_points= ExpandTrajParameters(parameters,x_curr[:3],x_trg[:3])
    score= EvalTraj(key_points, x_curr, x_trg, q_curr, N, ct, x_to_q_traj, to_ros_traj)
    #score= ModifyScoreFromExamples(
        #score= score,
        #bad_exs= ct.GetAttrOr([], 'memory','traj','bad'),
        #good_exs= ct.GetAttrOr([], 'memory','traj','good'),
        #dist_func= lambda situation, inferred_data, assessment: DistOfTrajExs(parameters, x_curr, x_trg, q_curr, N, ct, x_to_q_traj, to_ros_traj,  situation),
        #dist_threshold= 0.2)  #FIXME: dist_threshold is too big?

    #print score, parameters
    if score!=None:  return -score
    else:            return f_none


  #Lock the thread for self.mu (observer) like FK/IK in order to avoid
  #other threads (e.g. viz) access the FK service.
  #Otherwise, the FK process will die because the following code changes
  #the planning scene that the FK seems to be using.
  #This may be a bug reported:
  #http://answers.ros.org/question/10765/electric-get_fk-forward-kinematics-fails-simple-test/
  with ct.robot.sensor_locker:

    SetupScene()

    x_curr= ct.robot.FK(q=q_curr, x_ext=lw_x_ext, arm=arm)
    x_to_q_traj= lambda x_traj: \
      XTrajToQTraj(lambda x,q_start: ct.robot.IK(x, x_ext=lw_x_ext, start_angles=q_start, arm=arm),
                   x_traj, start_angles=q_curr)
      #ct.CartTrajToJointTraj(
        #[TransformRightInv(x,lw_x_ext) for x in x_traj],
        #dt=dt, start_angles=q_curr, arm=arm)
    to_ros_traj= lambda q_traj: ToROSTrajectory(ct.robot.JointNames(arm), q_traj, t_traj)

    #Check validity of start and goal
    vs,res1= ct.Run('scene','isvalidq',arm,[],q_curr)
    vg,res2,res3= ct.Run('scene','isvalidx',arm,TransformRightInv(x_trg,lw_x_ext),[],q_curr)

    if not vs:
      #CPrint(4,'Error: initial state is invalid:',res1.error_code.val)
      CPrint(4,'Error: initial state is invalid:',res1)
      #CPrint(4,'  Colliding objects:',GetCollidingObjects(res1.contacts))
      #CPrint(4,'  debug:',res1)
      viz.AddContacts(res1.contacts)
    if not vg:
      if res3 is None:
        CPrint(4,'Error: goal state is not reachable:',res2)
      else:
        #CPrint(4,'Warning: goal state seems to be invalid:',res3.error_code.val)
        CPrint(4,'Warning: goal state seems to be invalid:',res3)
        #CPrint(4,'  Colliding objects:',GetCollidingObjects(res3.contacts))
        viz.AddContacts(res3.contacts)
        if vs and ct.GetAttrOr(False, TMP,'conservative'):
          CPrint(4,'  Do you want to continue inference?')
          vg= AskYesNo()
    if not (vs and vg):
      CleanScene()
      return False

    #parameters_res= [math.pi*(-0.05), 1.0,math.pi*0.5, 1.1,math.pi*0.5]
    #parameters_res= [-0.0, 0.7,0.4, 0.3,0.5]

    fobj= lambda x: EvalTrajForFMin(x,x_curr,x_trg,q_curr,inum,ct,x_to_q_traj,to_ros_traj,None)
    options= {'CMA_diagonal':1, 'verb_time':0}
    options['bounds']= [
        [-0.5*math.pi, 0.0, 0.0,          0.0, 0.0],
        [ 0.5*math.pi, 2.0, 0.75*math.pi, 2.0, 0.75*math.pi]]
    options['tolfun']= 1.0e-2 # 1.0e-4
    options['maxfevals']= 200  #Rapid, but not acculate
    options['verb_log']= False
    options['scaling_of_variables']= np.array([1.0, 0.7,1.0, 0.7,1.0])
    parameters0= [0.0, 0.0,0.0, 0.0,0.0]
    scale0= 0.25*math.pi
    #parameters0= [math.pi*(-0.05), 1.0,math.pi*0.5, 1.1,math.pi*0.5]
    #scale0= 0.1

    options['scale0_ratio1']= 0.005
    options['scale0_ratio2']= 0.2
    options['init_guess_num']= 5
    options['max_init_guess_num']= [100,2000]
    options['db_search_num']= 5
    options['max_db_search_count']= 200
    options['db_param_novelty']= 0.1

    database= []  #We don't use database.
    parameters_res,score_res= MinimizeFunc_DB(fobj, parameters0, scale0, options,
                        database, db_search_key='key_points', infer_type='ver1')
    if score_res is None:
      CleanScene()
      CPrint(4,'No solution found')
      return False
    InsertDict(situation, {'infer_info':{'type':'ver1', 'param':parameters_res, 'score':score_res}})

    key_points= ExpandTrajParameters(parameters_res,x_curr[:3],x_trg[:3])
    ct.SetAttr(TMP,'key_points',  key_points)

    q_traj,x_traj= EvalTraj(key_points, x_curr, x_trg, q_curr, inum, ct, x_to_q_traj, to_ros_traj, return_traj=True)
    ct.SetAttr(TMP,'q_traj',  q_traj)
    ct.SetAttr(TMP,'t_traj',  t_traj)
    ct.SetAttr(TMP,'x_traj',  x_traj)

    #Visualize x_traj
    #viz= TSimpleVisualizer(rospy.Duration(10.0), name_space='visualizer_plan')
    viz.viz_frame= ct.robot.BaseFrame
    for n in range(len(x_traj)):
      x= x_traj[n]
      y= 0.2+0.8*float(n)/float(len(x_traj))
      rgb= [y,y,0.0]
      viz.AddCube(x, scale=[0.03,0.02,0.01], rgb=viz.ICol(5), alpha=0.5)
      viz.AddArrow(x, scale=[0.03,0.001,0.001], rgb=viz.ICol(5), alpha=0.5)

    CleanScene()

  return ct.HasAttr(TMP,'key_points') and ct.HasAttr(TMP,'q_traj') and ct.HasAttr(TMP,'t_traj')



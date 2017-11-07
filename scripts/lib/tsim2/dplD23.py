#!/usr/bin/python
from core_tool import *
SmartImportReload('lib.tsim.dpl_cmn')
from lib.tsim.dpl_cmn import *
def Help():
  return '''Dynamic Planning/Learning for grasping and pouring in ODE simulation
    using DPL version 4 (DNN, Bifurcation).
    Based on tsim2.dplD22, increased (unreasonable) selections.
    x WARNING: In this code we don'ct train DNN (as it's complicated).
    --> Implemented
    WARNING: Logging is incomplete.
  Usage: tsim2.dplD23'''

def Delta1(dim,s):
  assert(abs(s-int(s))<1.0e-6)
  p= [0.0]*dim
  p[int(s)]= 1.0
  return p

def Tlp_pour_dF(x_in,with_grad):  #Transform from ps_rcv,p_pour to lp_pour
  #In=['ps_rcv','p_pour']
  #Out=['lp_pour']
  ps_rcv,p_pour= x_in[:12],x_in[12:]
  pc_rcv= np.array(ps_rcv).reshape(4,3).mean(axis=0)  #Center of ps_rcv
  lp_pour= [ppx-pcx for ppx,pcx in zip(p_pour,pc_rcv)]

  if not with_grad:
    return lp_pour
  else:
    I= np.eye(3,3)
    I_4= -0.25*I
    D= np.zeros((15,3))
    D[0:3,:]= D[3:6,:]= D[6:9,:]= D[9:12,:]= I_4
    D[12:,:]= I
    return lp_pour, D

def Tlp_flow_dF(x_in,with_grad):  #Transform from ps_rcv,p_pour,lpp_flow to lp_flow2
  #In=['ps_rcv','p_pour','lpp_flow']
  #Out=['lp_flow2']
  ps_rcv,p_pour,lpp_flow= x_in[:12],x_in[12:15],x_in[15:]
  pc_rcv= np.array(ps_rcv).reshape(4,3).mean(axis=0)  #Center of ps_rcv
  lp_flow2= [ppx+lpp-pcx for ppx,lpp,pcx in zip(p_pour[:2],lpp_flow,pc_rcv[:2])]

  if not with_grad:
    return lp_flow2
  else:
    I= np.eye(3,2)
    I_4= -0.25*I
    D= np.zeros((17,2))
    D[0:3,:]= D[3:6,:]= D[6:9,:]= D[9:12,:]= I_4
    D[12:15,:]= I
    D[15:,:]= np.eye(2,2)
    return lp_flow2, D

def Execute(ct,l):
  ct.Run('tsim2.setup', l)
  sim= ct.sim
  #l= ct.sim_local

  actions={
    'grab'         : lambda a: ct.Run('tsim2.act.grab', a),
    'move_to_rcv'  : lambda a: ct.Run('tsim2.act.move_to_rcv', a),
    'move_to_pour' : lambda a: ct.Run('tsim2.act.move_to_pour', a),
    'std_pour'     : lambda a: ct.Run('tsim2.act.std_pour', a),
    'shake_A'      : lambda a: ct.Run('tsim2.act.shake_A', a),
    }

  #NOTE: Do not include 'da_trg' in obs_keys0 since 'da_trg' should be kept during some node transitions.
  obs_keys0= ('ps_rcv','p_pour','p_pour_z','lp_pour','a_trg','size_srcmouth','material2')
  obs_keys_after_grab= obs_keys0+('gh_abs',)
  obs_keys_before_flow= obs_keys_after_grab+('a_pour','a_spill2','a_total')
  obs_keys_after_flow= obs_keys_before_flow+('lp_flow2','lpp_flow','flow_var','da_pour','da_spill2','da_total')

  l.xs= TContainer()  #l.xs.NODE= XSSA
  l.idb= TContainer()  #l.idb.NODE= index in DB

  with sim.TPause(ct):  #Pause during plan/learn
    CPrint(2,'Node:','n0')
    l.xs.n0= ObserveXSSA(l,None,obs_keys0+('da_trg',))
    #TEST: Heuristic init guess
    pc_rcv= np.array(l.xs.n0['ps_rcv'].X).reshape(4,3).mean(axis=0)  #Center of ps_rcv
    l.xs.n0['p_pour_trg0']= SSA(Vec([-0.3,0.35])+Vec([pc_rcv[0],pc_rcv[2]]))  #A bit above of p_pour_trg
    res= l.dpl.Plan('n0', l.xs.n0)
    l.idb.n0= l.dpl.DB.AddToSeq(parent=None,name='n0',xs=l.xs.n0)
    l.xs.prev= l.xs.n0
    l.idb.prev= l.idb.n0

  select1= l.xs.n0['select1'].X[0]
  select3= l.xs.n0['select3'].X[0]

  gh_ratio= ToList(l.xs.n0['gh_ratio'].X)[0]
  actions['grab']({'gh_ratio':gh_ratio})
  with sim.TPause(ct):  #Pause during plan/learn
    l.xs.now= CopyXSSA(l.xs.prev)
    InsertDict(l.xs.now, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab))
    l.dpl.UpdateModel('Fgrasp',l.xs.prev,l.xs.now, not_learn=l.not_learn)
    l.xs.prev= l.xs.now

  if select1 in (0,3):
    p_pour_trg0= ToList(l.xs.n0['p_pour_trg0'].X)
    actions['move_to_rcv']({'p_pour_trg0':p_pour_trg0})
    with sim.TPause(ct):  #Pause during plan/learn
      l.xs.now= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.now, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab+('dps_rcv','v_rcv')))
      l.dpl.UpdateModel('Fmvtorcv2',l.xs.prev,l.xs.now, not_learn=l.not_learn)
      l.xs.prev= l.xs.now

  if (select1,select3)==(0,0) or select1==1:
    p_pour_trg= ToList(l.xs.n0['p_pour_trg'].X)
    actions['move_to_pour']({'p_pour_trg':p_pour_trg})
    with sim.TPause(ct):  #Pause during plan/learn
      l.xs.now= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.now, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab))
      l.dpl.UpdateModel('Fmvtopour3',l.xs.prev,l.xs.now, not_learn=l.not_learn)
      l.xs.prev= l.xs.now

  InsertDict(l.xs.prev, ObserveXSSA(l,l.xs.prev,obs_keys_before_flow))

  selected_skill= ('std_pour','shake_A')[l.xs.n0['skill'].X[0]]

  if selected_skill=='std_pour':
    dtheta1= l.xs.n0['dtheta1'].X[0,0]
    dtheta2= l.xs.n0['dtheta2'].X[0,0]
    actions['std_pour']({'dtheta1':dtheta1, 'dtheta2':dtheta2})
    with sim.TPause(ct):  #Pause during plan/learn
      l.xs.now= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.now, ObserveXSSA(l,l.xs.prev,obs_keys_after_flow))
      l.dpl.UpdateModel('Fflowc_tip11',l.xs.prev,l.xs.now, not_learn=l.not_learn)
      l.xs.prev= l.xs.now
      l.dpl.UpdateModel('Famount5',l.xs.prev,l.xs.now, not_learn=l.not_learn)

  elif selected_skill=='shake_A':
    dtheta1= l.xs.n0['dtheta1'].X[0,0]
    shake_spd= l.xs.n0['shake_spd'].X[0,0]
    shake_axis2= ToList(l.xs.n0['shake_axis2'].X)
    actions['shake_A']({'dtheta1':dtheta1, 'shake_spd':shake_spd, 'shake_axis2':shake_axis2})
    with sim.TPause(ct):  #Pause during plan/learn
      l.xs.now= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.now, ObserveXSSA(l,l.xs.prev,obs_keys_after_flow))
      l.dpl.UpdateModel('Fflowc_shakeA11',l.xs.prev,l.xs.now, not_learn=l.not_learn)
      l.xs.prev= l.xs.now
      l.dpl.UpdateModel('Famount5',l.xs.prev,l.xs.now, not_learn=l.not_learn)


def ConfigCallback(ct,l,sim):
  m_setup= ct.Load('tsim2.setup')

  l.amount_trg= 0.3
  #l.spilled_stop= 5
  l.spilled_stop= 10

  #l.config.RcvPos= [0.6, l.config.RcvPos[1], l.config.RcvPos[2]]
  l.config.RcvPos= [0.8+0.6*(random.random()-0.5), l.config.RcvPos[1], l.config.RcvPos[2]]
  CPrint(3,'l.config.RcvPos=',l.config.RcvPos)
  #l.config.ContactBounce= 0.1

  #InsertDict(l.config.__dict__, l.opt_conf['config'])
  for key,value in l.opt_conf['config'].iteritems():
    setattr(l.config, key, value)

  if l.rcv_size=='static':
    l.config.RcvSize= [0.3, 0.4, 0.2]
  elif l.rcv_size=='random':
    rsx= Rand(0.25,0.5)
    rsy= Rand(0.1,0.2)/rsx
    rsz= Rand(0.2,0.5)
    l.config.RcvSize= [rsx, rsy, rsz]

  if l.mtr_smsz=='fixed':
    m_setup.SetMaterial(l, preset='bounce')
    l.config.SrcSize2H= 0.03  #Mouth size of source container
  elif l.mtr_smsz=='fxvs1':
    m_setup.SetMaterial(l, preset='ketchup')
    l.config.SrcSize2H= 0.08  #Mouth size of source container
  elif l.mtr_smsz=='random':
    m_setup.SetMaterial(l, preset=('bounce','nobounce','natto','ketchup')[RandI(4)])
    l.config.SrcSize2H= Rand(0.02,0.09)  #Mouth size of source container
  elif l.mtr_smsz=='viscous':
    m_setup.SetMaterial(l, preset=('natto','ketchup')[RandI(2)])
    l.config.SrcSize2H= Rand(0.05,0.09)  #Mouth size of source container
  CPrint(3,'l.config.ViscosityParam1=',l.config.ViscosityParam1)
  CPrint(3,'l.config.SrcSize2H=',l.config.SrcSize2H)

def Run(ct,*args):
  l= TContainer(debug=True)
  l.logdir= args[0] if len(args)>0 else '/tmp/dpl/'
  opt_conf= args[1] if len(args)>1 else {}
  #l.planlearn_callback= PlanLearnCallback
  l.config_callback= ConfigCallback
  #l.m_sm= ct.Load('tsim.sm4')
  #Setup for experiments:
  l.logdir= '/tmp/dpl02/'
  opt_conf={
    'mtr_smsz': 'random',  #'fixed', 'fxvs1', 'random', 'viscous'
    'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
    #'model_dir': ct.DataBaseDir()+'models/tsim/v_exp1/',  #'',  Other than flow, amount
    #'model_dir': ct.DataBaseDir()+'models/tsim/v_exp4/',  #'',  Pre-trained w "fixed" (for dplD14)
    #'model_dir': ct.DataBaseDir()+'models/tsim/v_exp5/',  #'',  Pre-trained w "fixed" and "fxvs1" (for dplD14)
    #'model_dir': ct.DataBaseDir()+'models/tsim/v_exp6/',  #'',  Pre-trained w "fixed","fxvs1","random" (for dplD14)
    #'model_dir': '',
    #'model_dir_persistent': False,
    #'model_dir': ct.DataBaseDir()+'models/tsim2/v1/',  #'',  Trained with 9 episodes of ('random',None), trained with 60 episodes of ('fixed','early_tip'), trained with 30 episodes of ('fxvs1','early_shakeA'), trained with 60 episodes of ('random',None)
    'model_dir': ct.DataBaseDir()+'models/tsim2/v2/',  #'',  Trained with 42 episodes of ('random',None) with dplD23 setup (extended graph)
    'model_dir_persistent': False,
    }

  l.opt_conf={
    'interactive': True,
    'not_learn': False,
    'num_episodes': 1e+6,
    'num_log_interval': 3,
    'rcv_size': 'static',  #'static', 'random'
    'mtr_smsz': 'random',  #'fixed', 'fxvs1', 'random', 'viscous'
    'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
    'model_dir': ct.DataBaseDir()+'models/tsim2/v1/',  #'',
    'model_dir_persistent': True,  #If False, models are saved in l.logdir, i.e. different one from 'model_dir'
    'db_src': '',
    #'db_src': '/tmp/dpl/database.yaml',
    'config': {},  #Config of the simulator
    'dpl_options': {
      'opt_log_name': None,  #Not save optimization log.
      },
    }
  InsertDict(l.opt_conf, opt_conf)

  l.interactive= l.opt_conf['interactive']
  l.num_episodes= l.opt_conf['num_episodes']
  l.num_log_interval= l.opt_conf['num_log_interval']
  l.rcv_size= l.opt_conf['rcv_size']
  l.mtr_smsz= l.opt_conf['mtr_smsz']
  l.rwd_schedule= l.opt_conf['rwd_schedule']

  l.not_learn= l.opt_conf['not_learn']
  #l.not_learn= True  #Models are not trained.

  #Learning scheduling
  def EpisodicCallback(l,count):
    Rdamount_default= [['da_pour','da_trg','da_spill2'],[REWARD_KEY],
          TLocalQuad(3,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 1.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2)]
    Rdamount_early_tip= [['da_pour','da_trg','da_spill2','skill'],[REWARD_KEY],
          TLocalQuad(4,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 1.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2 - (10.0 if y[3]!=0 else 0.0))]
    Rdamount_early_shakeA= [['da_pour','da_trg','da_spill2','skill'],[REWARD_KEY],
          TLocalQuad(4,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 1.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2 - (10.0 if y[3]!=1 else 0.0))]
    #'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
    if l.rwd_schedule is None:
      #No reward scheduling
      pass
    elif l.rwd_schedule=='early_tip':
      #Reward scheduling (FOR EARLY TIPPING)
      if count<10:  l.dpl.d.Models['Rdamount']= Rdamount_early_tip
      else:         l.dpl.d.Models['Rdamount']= Rdamount_default
    elif l.rwd_schedule=='early_shakeA':
      #Reward scheduling (FOR EARLY SHAKING-A)
      if count<10:  l.dpl.d.Models['Rdamount']= Rdamount_early_shakeA
      else:         l.dpl.d.Models['Rdamount']= Rdamount_default

  def LogDPL(l):
    #SaveYAML(l.dpl.MM.Save(l.dpl.MM.Options['base_dir']), l.dpl.MM.Options['base_dir']+'model_mngr.yaml')
    l.mm.Save()
    SaveYAML(l.dpl.DB.Save(), l.logdir+'database.yaml')
    SaveYAML(l.dpl.Save(), l.logdir+'dpl.yaml')

    #'''
    #Analyze l.dpl.DB.Entry:
    ptree= l.dpl.GetPTree('n0', {})
    fp= open(l.logdir+'dpl_est.dat','w')
    for i,eps in enumerate(l.dpl.DB.Entry):
      n0_0= eps.Find(('n0',0))[0]
      if n0_0 is None or eps.R is None:
        CPrint(4, 'l.dpl.DB has a broken entry')
        continue
      ptree.StartNode.XS= n0_0.XS
      ptree.ResetFlags()
      values= [eps.R, l.dpl.Value(ptree)]
      fp.write('%i %s\n' % (i, ' '.join(map(str,values))))
    fp.close()
    CPrint(1,'Generated:',l.logdir+'dpl_est.dat')
    #'''

  if l.interactive and 'log_dpl' in ct.__dict__ and (CPrint(1,'Restart from existing DPL?'), AskYesNo())[1]:
    l.dpl= ct.log_dpl
    l.mm= ct.log_mm
    l.restarting= True
  else:
    #Setup dynamic planner/learner
    domain= TGraphDynDomain()
    SP= TCompSpaceDef
    domain.SpaceDefs={
      'skill': SP('select',num=2),  #Skill selection
      'select1': SP('select',num=3),  #Selection at n1
      'select3': SP('select',num=2),  #Selection at n3
      'ps_rcv': SP('state',12),  #4 edge point positions (x,y,z)*4 of receiver
      'gh_ratio': SP('action',1,min=[0.0],max=[1.0]),  #Gripper height (ratio)
      'gh_abs': SP('state',1),  #Gripper height (absolute value)
      'p_pour_trg0': SP('state',2,min=[0.2,0.1],max=[1.2,0.7]),  #Target pouring axis position of preparation before pouring (x,z)
        #NOTE: we stopped to plan p_pour_trg0
      'p_pour_trg': SP('action',2,min=[0.2,0.1],max=[1.2,0.7]),  #Target pouring axis position (x,z)
      'dtheta1': SP('action',1,min=[0.01],max=[0.02]),  #Pouring skill parameter for all skills
      'dtheta2': SP('action',1,min=[0.002],max=[0.005]),  #Pouring skill parameter for 'std_pour'
      #'dtheta1': SP('state',1),  #Pouring skill parameter for all skills
      #'dtheta2': SP('state',1),  #Pouring skill parameter for 'std_pour'
      'shake_spd': SP('action',1,min=[0.7],max=[0.9]),  #Pouring skill parameter for 'shake_A'
      #'shake_spd': SP('state',1),  #Pouring skill parameter for 'shake_A'
      #'shake_axis': SP('action',2,min=[0.0,0.0],max=[0.1,0.1]),  #Pouring skill parameter for 'shake_A'
      'shake_axis2': SP('action',2,min=[0.05,-0.5*math.pi],max=[0.1,0.5*math.pi]),  #Pouring skill parameter for 'shake_A'
      #'shake_axis2': SP('state',2),  #Pouring skill parameter for 'shake_A'
      'p_pour': SP('state',3),  #Pouring axis position (x,y,z)
      'p_pour_z': SP('state',1),  #Pouring axis position (z)
      'lp_pour': SP('state',3),  #Pouring axis position (x,y,z) in receiver frame
      'dps_rcv': SP('state',12),  #Displacement of ps_rcv from previous time
      'v_rcv': SP('state',1),  #Velocity norm of receiver
      #'p_flow': SP('state',2),  #Flow position (x,y)
      #'lp_flow': SP('state',2),  #Flow position (x,y) in receiver frame
      'lp_flow2': SP('state',2),  #Flow position (x,y) in receiver frame (no atan)
      'lpp_flow': SP('state',2),  #Flow position (x,y) relative to previous (before flowctrl) p_pour
      'flow_var': SP('state',1),  #Variance of flow
      'a_pour': SP('state',1),  #Amount poured in receiver
      'a_spill2': SP('state',1),  #Amount spilled out
      'a_total':  SP('state',1),  #Total amount moved from source
      'a_trg': SP('state',1),  #Target amount
      'da_pour': SP('state',1),  #Amount poured in receiver (displacement)
      'da_spill2': SP('state',1),  #Amount spilled out (displacement)
      'da_total':  SP('state',1),  #Total amount moved from source (displacement)
      'da_trg': SP('state',1),  #Target amount (displacement)
      'size_srcmouth': SP('state',1),  #Size of mouth of the source container
      'material2': SP('state',4),  #Material property (e.g. viscosity)
      REWARD_KEY:  SP('state',1),
      }

    mm_options= {
      #'type': 'lwr',
      'load_dir': l.logdir+'models/',
      'save_dir': l.logdir+'models/',
      }
    mm= TModelManager2(domain.SpaceDefs)
    mm.Load({'options':mm_options})
    if l.opt_conf['model_dir'] not in ('',None):
      mm.Options['load_dir']= l.opt_conf['model_dir']
      if l.opt_conf['model_dir_persistent']:  mm.Options['save_dir']= l.opt_conf['model_dir']
      else:                                   mm.Options['save_dir']= mm_options['save_dir']

    domain.Models={
      #key:[In,Out,F],
      'Fnone': [[],[], None],
      'Fgrasp': mm.Learner('Fgrasp',['gh_ratio'],['gh_abs']),  #Grasping. NOTE: removed ps_rcv
      'Fmvtorcv2': mm.Combined(  #Move to receiver; NOTE: unified with Fmvtorcv_rcvmv
        mm.Learner('Fmvtorcv2_ps_rcv',
          ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
          ['ps_rcv']),
        mm.Learner('Fmvtorcv2_p_pour',
          ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
          ['p_pour']),
        mm.Learner('Fmvtorcv2_dps_rcv',
          ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
          ['dps_rcv']),
        mm.Learner('Fmvtorcv2_v_rcv',
          ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
          ['v_rcv']),  ),
      'Fmvtopour3': mm.Combined(  #Move to pouring point
        mm.Learner('Fmvtopour3_ps_rcv',
          ['ps_rcv','gh_abs','p_pour','p_pour_trg'],
          ['ps_rcv']),  #NOTE: original output was 'lp_pour' (now is separated into this and Tlp_pour)
        mm.Learner('Fmvtopour3_p_pour',
          ['ps_rcv','gh_abs','p_pour','p_pour_trg'],
          ['p_pour']),  ),
      'Tp_pour_z': #Transform from p_pour to p_pour_z
        mm.Manual(['p_pour'],['p_pour_z'],
          TLocalLinear(3,1,lambda x:[x[2]],lambda x:[[0.0],[0.0],[1.0]])),
      'Fflowc_tip11': mm.Learner('Fflowc_tip11',  #Flow control with tipping.
        ['gh_abs','p_pour_z',  #Removed 'lp_pour','p_pour_trg0','p_pour_trg'; Added 'p_pour_z' (z is important for 'lpp_flow')
        'da_trg','size_srcmouth','material2',
        'dtheta1','dtheta2'],
        ['da_total','lpp_flow','flow_var']),  #Removed 'p_pour'
      'Fflowc_shakeA11': mm.Learner('Fflowc_shakeA11',  #Flow control with shake_A.
        ['gh_abs','p_pour_z',  #Removed 'lp_pour','p_pour_trg0','p_pour_trg'; Added 'p_pour_z' (z is important for 'lpp_flow')
        'da_trg','size_srcmouth','material2',
        'dtheta1','shake_spd','shake_axis2'],
        ['da_total','lpp_flow','flow_var']),  #Removed 'p_pour'
      #'Tlp_pour': #Transform from ps_rcv,p_pour to lp_pour
        #mm.Manual(['ps_rcv','p_pour'],['lp_pour'],
          #TLocalLinear(15,3,FdF=Tlp_pour_dF)),
      'Tlp_flow':  #Transform from ps_rcv,p_pour,lpp_flow to lp_flow2
        mm.Manual(['ps_rcv','p_pour','lpp_flow'],['lp_flow2'],
          TLocalLinear(17,2,FdF=Tlp_flow_dF)),
      ##TODO: make analytically combined function of Trcv:
      #'Trcv': mm.Combined( #Transform ps_rcv,p_pour,lpp_flow to lp_pour,lp_flow2
        #mm.Manual(['ps_rcv','p_pour'],['lp_pour'],
          #TLocalLinear(15,3,FdF=Tlp_pour_dF)),
        #mm.Manual(['ps_rcv','p_pour','lpp_flow'],['lp_flow2'],
          #TLocalLinear(17,2,FdF=Tlp_flow_dF)),  ),
      'Famount5': mm.Learner('Famount5',  #Amount model common for tip and shake.
        [#Removed 'lp_pour','gh_abs','p_pour_trg0','p_pour_trg'; NOTE-2: Now p_pour_z is used to estimate lp_flow2, which would be enough to optimize p_pour_trg.  NOTE-1: Don'ct remove 'lp_pour' otherwise there is no information to optimize z of p_pour_trg.
        #Removed 'material2','da_trg','size_srcmouth'
        'da_total','lp_flow2','flow_var'],
        ['da_pour','da_spill2']),
      'Rrcvmv':  mm.Manual(['dps_rcv','v_rcv'],[REWARD_KEY],TLocalQuad(13,lambda y:-(np.dot(y[:12],y[:12]) + y[12]*y[12]))),
      'Rmvtopour':  mm.Manual(['p_pour_trg','p_pour'],[REWARD_KEY],TLocalQuad(5,lambda y:-0.1*((y[0]-y[2])**2+(y[1]-y[4])**2))),
      #'Ramount':  mm.Manual(['a_pour','a_trg','a_spill2'],[REWARD_KEY],TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - y[2]*y[2])),
      #'Rdamount':  mm.Manual(['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                    #TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - y[2]*y[2])),
      #'Rdamount':  mm.Manual(['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                    #TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - math.log(1.0+max(0.0,y[2])))),
      #'Rdamount':  mm.Manual(['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                    #TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - max(0.0,y[2])**2)),
      'Rdamount':  mm.Manual(['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                    TLocalQuad(3,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 1.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2)),
      #'Rdamount':  mm.Manual(['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                    #TLocalQuad(3,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 10.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2)),
      'P1': mm.Manual([],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])),
      'P2':  mm.Manual([],[PROB_KEY], TLocalLinear(0,2,lambda x:[1.0]*2,lambda x:[0.0]*2)),
      'Pskill': mm.Manual(['skill'],[PROB_KEY], TLocalLinear(0,2,lambda s:Delta1(2,s[0]),lambda s:[0.0]*2)),
      'Pselect1': mm.Manual(['select1'],[PROB_KEY], TLocalLinear(0,3,lambda s:Delta1(3,s[0]),lambda s:[0.0]*3)),
      'Pselect3': mm.Manual(['select3'],[PROB_KEY], TLocalLinear(0,2,lambda s:Delta1(2,s[0]),lambda s:[0.0]*2)),
      }
    domain.Graph={
      #n0->'Fgrasp'->n1->'Fmvtorcv2'->n2->'Fnone'->n3->'Fmvtopour3' A/
      #                               n2->'Rrcvmv'->n2r
      #                                            n3->'Tp_pour_z'->n54 B4/
      #              n1->'Fnone'->n32->'Fmvtopour3' A2/
      #              n1->'Fnone'->n53 B3/
      #/A->n4->'Tp_pour_z'->n5 B/
      #    n4->'Rmvtopour'->n4r
      #/B->'Fflowc_tip11'   ->n6ti->'Tlp_flow'->n7ti->'Famount5'->n8ti->'Rdamount'->n8tir
      #/B->'Fflowc_shakeA11'->n6sa->'Tlp_flow'->n7sa->'Famount5'->n8sa->'Rdamount'->n8sar
      #
      #/A2->n42->'Tp_pour_z'->n52 B2/
      #     n42->'Rmvtopour'->n42r
      #/B2->'Fflowc_tip11'   ->n62ti->'Tlp_flow'->n72ti->'Famount5'->n82ti->'Rdamount'->n82tir
      #/B2->'Fflowc_shakeA11'->n62sa->'Tlp_flow'->n72sa->'Famount5'->n82sa->'Rdamount'->n82sar
      #
      #/B3->'Fflowc_tip11'   ->n63ti->'Tlp_flow'->n73ti->'Famount5'->n83ti->'Rdamount'->n83tir
      #/B3->'Fflowc_shakeA11'->n63sa->'Tlp_flow'->n73sa->'Famount5'->n83sa->'Rdamount'->n83sar
      #
      #/B4->'Fflowc_tip11'   ->n64ti->'Tlp_flow'->n74ti->'Famount5'->n84ti->'Rdamount'->n84tir
      #/B4->'Fflowc_shakeA11'->n64sa->'Tlp_flow'->n74sa->'Famount5'->n84sa->'Rdamount'->n84sar
      'n0': TDynNode(None,'P1',('Fgrasp','n1')),
      'n1': TDynNode('n0','Pselect1',('Fmvtorcv2','n2'),('Fnone','n32'),('Fnone','n53')),
      'n2': TDynNode('n1','P2',('Fnone','n3'),('Rrcvmv','n2r')),
      'n2r': TDynNode('n2'),
      'n3': TDynNode('n2','Pselect3',('Fmvtopour3','n4'),('Tp_pour_z','n54')),
      'n4': TDynNode('n3','P2',('Tp_pour_z','n5'),('Rmvtopour','n4r')),
      'n4r': TDynNode('n4'),
      'n5': TDynNode('n4','Pskill',('Fflowc_tip11','n6ti'),('Fflowc_shakeA11','n6sa')),

      'n32': TDynNode('n1','P1',('Fmvtopour3','n42')),
      'n42': TDynNode('n32','P2',('Tp_pour_z','n52'),('Rmvtopour','n42r')),
      'n42r': TDynNode('n42'),
      'n52': TDynNode('n42','Pskill',('Fflowc_tip11','n62ti'),('Fflowc_shakeA11','n62sa')),

      'n53': TDynNode('n1','Pskill',('Fflowc_tip11','n63ti'),('Fflowc_shakeA11','n63sa')),
      'n54': TDynNode('n3','Pskill',('Fflowc_tip11','n64ti'),('Fflowc_shakeA11','n64sa')),
      #Tipping:
      'n6ti': TDynNode('n5','P1',('Tlp_flow','n7ti')),
      'n7ti': TDynNode('n6ti','P1',('Famount5','n8ti')),
      'n8ti': TDynNode('n7ti','P1',('Rdamount','n8tir')),
      'n8tir': TDynNode('n8ti'),

      'n62ti': TDynNode('n52','P1',('Tlp_flow','n72ti')),
      'n72ti': TDynNode('n62ti','P1',('Famount5','n82ti')),
      'n82ti': TDynNode('n72ti','P1',('Rdamount','n82tir')),
      'n82tir': TDynNode('n82ti'),

      'n63ti': TDynNode('n53','P1',('Tlp_flow','n73ti')),
      'n73ti': TDynNode('n63ti','P1',('Famount5','n83ti')),
      'n83ti': TDynNode('n73ti','P1',('Rdamount','n83tir')),
      'n83tir': TDynNode('n83ti'),

      'n64ti': TDynNode('n54','P1',('Tlp_flow','n74ti')),
      'n74ti': TDynNode('n64ti','P1',('Famount5','n84ti')),
      'n84ti': TDynNode('n74ti','P1',('Rdamount','n84tir')),
      'n84tir': TDynNode('n84ti'),
      #Shaking-A:
      'n6sa': TDynNode('n5','P1',('Tlp_flow','n7sa')),
      'n7sa': TDynNode('n6sa','P1',('Famount5','n8sa')),
      'n8sa': TDynNode('n7sa','P1',('Rdamount','n8sar')),
      'n8sar': TDynNode('n8sa'),

      'n62sa': TDynNode('n52','P1',('Tlp_flow','n72sa')),
      'n72sa': TDynNode('n62sa','P1',('Famount5','n82sa')),
      'n82sa': TDynNode('n72sa','P1',('Rdamount','n82sar')),
      'n82sar': TDynNode('n82sa'),

      'n63sa': TDynNode('n53','P1',('Tlp_flow','n73sa')),
      'n73sa': TDynNode('n63sa','P1',('Famount5','n83sa')),
      'n83sa': TDynNode('n73sa','P1',('Rdamount','n83sar')),
      'n83sar': TDynNode('n83sa'),

      'n64sa': TDynNode('n54','P1',('Tlp_flow','n74sa')),
      'n74sa': TDynNode('n64sa','P1',('Famount5','n84sa')),
      'n84sa': TDynNode('n74sa','P1',('Rdamount','n84sar')),
      'n84sar': TDynNode('n84sa'),
      }

    db= TGraphEpisodeDB()
    if l.opt_conf['db_src'] not in ('',None):
      db.Load(LoadYAML(l.opt_conf['db_src']))

    l.dpl= TGraphDynPlanLearn2(domain, db)
    l.mm= mm
    l.restarting= False

  dpl_options={
    'base_dir': l.logdir,
    }
  InsertDict(dpl_options, l.opt_conf['dpl_options'])
  l.dpl.Load({'options':dpl_options})


  ct.log_dpl= l.dpl  #for log purpose
  ct.log_mm= l.mm

  print 'Copying',PycToPy(__file__),'to',PycToPy(l.logdir+os.path.basename(__file__))
  CopyFile(PycToPy(__file__),PycToPy(l.logdir+os.path.basename(__file__)))

  count= 0
  if l.restarting:
    fp= OpenW(l.logdir+'dpl_log.dat','a')
  else:
    fp= OpenW(l.logdir+'dpl_log.dat','w')
    if len(l.dpl.DB.Entry)>0:
      for i in range(len(l.dpl.DB.Entry)):
        fp.write(l.dpl.DB.DumpOneYAML(i))
      fp.flush()
  while True:
    for i in range(l.num_log_interval):
      CPrint(2,'========== Start %4i =========='%count)
      EpisodicCallback(l,count)
      l.dpl.NewEpisode()
      l.user_viz= []
      try:
        Execute(ct,l)
      finally:
        ct.sim.StopPubSub(ct,l)
        ct.sim_local.sensor_callback= None
        ct.srvp.ode_pause()
      l.dpl.EndEpisode()
      CPrint(2,'========== End %4i =========='%count)
      #xyar_line= l.dpl.DB.Entry[-1].Dump()
      fp.write(l.dpl.DB.DumpOneYAML())
      fp.flush()
      CPrint(1,count,l.dpl.DB.DumpOne())
      count+= 1
      if count>=l.num_episodes:  break
    LogDPL(l)
    if count>=l.num_episodes:  break
    if l.interactive:
      print 'Continue?'
      if not AskYesNo():  break
  fp.close()

  l= None
  return True

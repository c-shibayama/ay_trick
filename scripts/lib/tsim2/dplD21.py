#!/usr/bin/python
from core_tool import *
SmartImportReload('lib.tsim.dpl_cmn')
from lib.tsim.dpl_cmn import *
def Help():
  return '''Dynamic Planning/Learning for grasping and pouring in ODE simulation
    using DPL version 4 (DNN, Bifurcation).
    The behavior is the same as tsim2.dplD20,
    replaced the model manager and the DPL utility by TModelManager2 and TGraphDynPlanLearn2.
    We share the same model Famount_* in different skills.
  Usage: tsim2.dplD21'''

def Delta1(dim,s):
  assert(abs(s-int(s))<1.0e-6)
  p= [0.0]*dim
  p[int(s)]= 1.0
  return p

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
  obs_keys0= ('ps_rcv','p_pour','lp_pour','a_trg','size_srcmouth','material2')
  obs_keys_after_grab= obs_keys0+('gh_abs',)
  obs_keys_before_flow= obs_keys_after_grab+('a_pour','a_spill2','a_total')
  obs_keys_after_flow= obs_keys_before_flow+('lp_flow','flow_var','da_pour','da_spill2','da_total')

  l.xs= TContainer()  #l.xs.NODE= XSSA
  l.idb= TContainer()  #l.idb.NODE= index in DB

  with sim.TPause(ct):  #Pause during plan/learn
    CPrint(2,'Node:','n0')
    l.xs.n0= ObserveXSSA(l,None,obs_keys0+('da_trg',))
    #TEST: Heuristic init guess
    pc_rcv= np.array(l.xs.n0['ps_rcv'].X).reshape(4,3).mean(axis=0)  #Center of ps_rcv
    #l.xs.n0['gh_ratio']= SSA([0.5])
    #l.xs.n0['p_pour_trg']= SSA(Vec([0.0,0.1])+Vec([pc_rcv[0],pc_rcv[2]]))  #A bit above of Center of ps_rcv
    l.xs.n0['p_pour_trg0']= SSA(Vec([-0.3,0.35])+Vec([pc_rcv[0],pc_rcv[2]]))  #A bit above of p_pour_trg
    #l.xs.n0['dtheta1']= SSA([0.014])
    #l.xs.n0['dtheta2']= SSA([0.004])
    #l.xs.n0['shake_spd']= SSA([0.8])
    #l.xs.n0['shake_axis2']= SSA([0.08,0.0])
    #l.xs.n0['skill']= SSA([1])
    res= l.dpl.Plan('n0', l.xs.n0)
    l.idb.n0= l.dpl.DB.AddToSeq(parent=None,name='n0',xs=l.xs.n0)
    l.xs.prev= l.xs.n0
    l.idb.prev= l.idb.n0

  gh_ratio= ToList(l.xs.n0['gh_ratio'].X)[0]
  actions['grab']({'gh_ratio':gh_ratio})

  with sim.TPause(ct):  #Pause during plan/learn
    #Plan l.p_pour_trg0, l.theta_init
    CPrint(2,'Node:','n1')
    l.xs.n1= CopyXSSA(l.xs.prev)
    InsertDict(l.xs.n1, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab))
    l.dpl.UpdateModel('Fgrasp',l.xs.prev,l.xs.n1, not_learn=l.not_learn)
    #res= l.dpl.Plan('n1', l.xs.n1)
    l.idb.n1= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n1',xs=l.xs.n1)
    l.xs.prev= l.xs.n1
    l.idb.prev= l.idb.n1

  p_pour_trg0= ToList(l.xs.n1['p_pour_trg0'].X)
  p_pour_trg= ToList(l.xs.n1['p_pour_trg'].X)
  actions['move_to_rcv']({'p_pour_trg0':p_pour_trg0})
  VizPP(l,[p_pour_trg0[0],0.0,p_pour_trg0[1]],[0.,1.,0.])
  VizPP(l,[p_pour_trg[0],0.0,p_pour_trg[1]],[0.5,0.,1.])

  with sim.TPause(ct):  #Pause during plan/learn
    #Branch-1: reward
    CPrint(2,'Node:','n1rcvmv')
    l.xs.n1rcvmv= CopyXSSA(l.xs.prev)
    InsertDict(l.xs.n1rcvmv, ObserveXSSA(l,l.xs.prev,('dps_rcv','v_rcv')))
    l.dpl.UpdateModel('Fmvtorcv_rcvmv',l.xs.prev,l.xs.n1rcvmv, not_learn=l.not_learn)
    #res= l.dpl.Plan('n1rcvmv', l.xs.n1rcvmv)
    l.idb.n1rcvmv= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n1rcvmv',xs=l.xs.n1rcvmv)

    CPrint(2,'Node:','n1rcvmvr')
    #Since we have 'Rrcvmv', we just use it to get the next XSSA
    l.xs.n1rcvmvr= l.dpl.Forward('Rrcvmv',l.xs.n1rcvmv)
    l.idb.n1rcvmvr= l.dpl.DB.AddToSeq(parent=l.idb.n1rcvmv,name='n1rcvmvr',xs=l.xs.n1rcvmvr)

    #Branch-2: main procedure
    CPrint(2,'Node:','n2a','(update)')
    l.xs.n2a= CopyXSSA(l.xs.prev)
    InsertDict(l.xs.n2a, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab))
    l.dpl.UpdateModel('Fmvtorcv',l.xs.prev,l.xs.n2a, not_learn=l.not_learn)

  repeated= False  #For try-and-error learning
  while True:  #Try-and-error starts from here.
    #Three cases of parent of l.idb.n2a: l.idb.n1, l.idb.n4ti, l.idb.n4sa

    with sim.TPause(ct):  #Pause during plan/learn
      #Plan l.p_pour_trg
      CPrint(2,'Node:','n2a','(plan)')
      l.xs.n2a= CopyXSSA(l.xs.prev)
      if repeated:
        #Delete actions and selections (e.g. skill) to plan again from initial guess.
        for key in l.xs.n2a.keys():
          if l.dpl.d.SpaceDefs[key].Type in ('action','select'):
            del l.xs.n2a[key]
      InsertDict(l.xs.n2a, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab+('da_trg',)))
      #TEST: Heuristic init guess
      #l.xs.n2a['skill']= SSA([1])
      res= l.dpl.Plan('n2a', l.xs.n2a)
      l.idb.n2a= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n2a',xs=l.xs.n2a)
      l.xs.prev= l.xs.n2a
      l.idb.prev= l.idb.n2a

    p_pour_trg= ToList(l.xs.n2a['p_pour_trg'].X)
    actions['move_to_pour']({'p_pour_trg':p_pour_trg})
    l.user_viz.pop()
    VizPP(l,[p_pour_trg[0],0.0,p_pour_trg[1]],[1.,0.,1.])


    with sim.TPause(ct):  #Pause during plan/learn
      CPrint(2,'Node:','n2b')
      l.xs.n2b= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.n2b, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab))
      l.dpl.UpdateModel('Fmvtopour2',l.xs.prev,l.xs.n2b, not_learn=l.not_learn)
      #res= l.dpl.Plan('n2b', l.xs.n2b)
      l.idb.n2b= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n2b',xs=l.xs.n2b)
      l.xs.prev= l.xs.n2b
      l.idb.prev= l.idb.n2b

    #Branch-1: main procedure
    #Just go to 'n2c'

    #Branch-2: reward
    CPrint(2,'Node:','n2br')
    #Since we have 'Rmvtopour', we just use it to get the next XSSA
    l.xs.n2br= l.dpl.Forward('Rmvtopour',l.xs.prev)
    l.idb.n2br= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n2br',xs=l.xs.n2br)

    with sim.TPause(ct):  #Pause during plan/learn
      #Plan l.selected_skill from ('std_pour','shake_A','shake_B')
      CPrint(2,'Node:','n2c')
      l.xs.n2c= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.n2c, ObserveXSSA(l,l.xs.prev,obs_keys_before_flow))
      #l.dpl.UpdateModel('Fnone',l.xs.prev,l.xs.n2c, not_learn=l.not_learn)
      #res= l.dpl.Plan('n2c', l.xs.n2c)
      l.idb.n2c= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n2c',xs=l.xs.n2c)
      l.xs.prev= l.xs.n2c
      l.idb.prev= l.idb.n2c

    selected_skill= ('std_pour','shake_A')[l.xs.n2c['skill'].X[0]]
    #selected_skill= 'shake_A'
    if selected_skill=='std_pour':
      dtheta1= l.xs.n2c['dtheta1'].X[0,0]
      dtheta2= l.xs.n2c['dtheta2'].X[0,0]
      actions['std_pour']({'dtheta1':dtheta1, 'dtheta2':dtheta2})

      with sim.TPause(ct):  #Pause during plan/learn
        CPrint(2,'Node:','n3ti')
        l.xs.n3ti= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n3ti, ObserveXSSA(l,l.xs.prev,obs_keys_after_flow))
        l.dpl.UpdateModel('Fflowc_tip10',l.xs.prev,l.xs.n3ti, not_learn=l.not_learn)
        #res= l.dpl.Plan('n3ti', l.xs.n3ti)
        l.idb.n3ti= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n3ti',xs=l.xs.n3ti)
        l.xs.prev= l.xs.n3ti
        l.idb.prev= l.idb.n3ti

        CPrint(2,'Node:','n4ti')
        l.xs.n4ti= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n4ti, ObserveXSSA(l,l.xs.prev,()))  #Observation is omitted since there is no change
        #WARNING:NOTE: Famount4 uses 'lp_pour' as input, so here we use a trick:
        xs_in= CopyXSSA(l.xs.prev)
        xs_in['lp_pour']= l.xs.n2c['lp_pour']
        #l.dpl.UpdateModel('Famount4',l.xs.prev,l.xs.n4ti, not_learn=l.not_learn)
        l.dpl.UpdateModel('Famount4',xs_in,l.xs.n4ti, not_learn=l.not_learn)
        #res= l.dpl.Plan('n4ti', l.xs.n4ti)
        l.idb.n4ti= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n4ti',xs=l.xs.n4ti)
        l.xs.prev= l.xs.n4ti
        l.idb.prev= l.idb.n4ti

        CPrint(2,'Node:','n4tir')
        l.xs.n4tir= l.dpl.Forward('Rdamount',l.xs.prev)
        l.idb.n4tir= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n4tir',xs=l.xs.n4tir)

    elif selected_skill=='shake_A':
      dtheta1= l.xs.n2c['dtheta1'].X[0,0]
      shake_spd= l.xs.n2c['shake_spd'].X[0,0]
      shake_axis2= ToList(l.xs.n2c['shake_axis2'].X)
      actions['shake_A']({'dtheta1':dtheta1, 'shake_spd':shake_spd, 'shake_axis2':shake_axis2})

      with sim.TPause(ct):  #Pause during plan/learn
        CPrint(2,'Node:','n3sa')
        l.xs.n3sa= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n3sa, ObserveXSSA(l,l.xs.prev,obs_keys_after_flow))
        l.dpl.UpdateModel('Fflowc_shakeA10',l.xs.prev,l.xs.n3sa, not_learn=l.not_learn)
        #res= l.dpl.Plan('n3sa', l.xs.n3sa)
        l.idb.n3sa= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n3sa',xs=l.xs.n3sa)
        l.xs.prev= l.xs.n3sa
        l.idb.prev= l.idb.n3sa

        CPrint(2,'Node:','n4sa')
        l.xs.n4sa= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n4sa, ObserveXSSA(l,l.xs.prev,()))  #Observation is omitted since there is no change
        #WARNING:NOTE: Famount4 uses 'lp_pour' as input, so here we use a trick:
        xs_in= CopyXSSA(l.xs.prev)
        xs_in['lp_pour']= l.xs.n2c['lp_pour']
        #l.dpl.UpdateModel('Famount4',l.xs.prev,l.xs.n4sa, not_learn=l.not_learn)
        l.dpl.UpdateModel('Famount4',xs_in,l.xs.n4sa, not_learn=l.not_learn)
        #res= l.dpl.Plan('n4sa', l.xs.n4sa)
        l.idb.n4sa= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n4sa',xs=l.xs.n4sa)
        l.xs.prev= l.xs.n4sa
        l.idb.prev= l.idb.n4sa

        CPrint(2,'Node:','n4sar')
        l.xs.n4sar= l.dpl.Forward('Rdamount',l.xs.prev)
        l.idb.n4sar= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n4sar',xs=l.xs.n4sar)

    # Conditions to break the try-and-error loop
    if l.IsPoured():
      break
    if l.IsTimeout() or l.IsEmpty():  # or l.IsSpilled()
      break
    if not IsSuccess(l.exec_status):
      break

    repeated= True



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
  l.logdir= '/tmp/dpl01/'
  opt_conf={
    'mtr_smsz': 'random',  #'fixed', 'fxvs1', 'random', 'viscous'
    'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
    #'model_dir': ct.DataBaseDir()+'models/tsim/v_exp1/',  #'',  Other than flow, amount
    'model_dir': ct.DataBaseDir()+'models/tsim/v_exp4/',  #'',  Pre-trained w "fixed" (for dplD14)
    #'model_dir': ct.DataBaseDir()+'models/tsim/v_exp5/',  #'',  Pre-trained w "fixed" and "fxvs1" (for dplD14)
    #'model_dir': ct.DataBaseDir()+'models/tsim/v_exp6/',  #'',  Pre-trained w "fixed","fxvs1","random" (for dplD14)
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
    'model_dir': ct.DataBaseDir()+'models/tsim/v2/',  #'',
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
      'lp_pour': SP('state',3),  #Pouring axis position (x,y,z) in receiver frame
      'dps_rcv': SP('state',12),  #Displacement of ps_rcv from previous time
      'v_rcv': SP('state',1),  #Velocity norm of receiver
      #'p_flow': SP('state',2),  #Flow position (x,y)
      'lp_flow': SP('state',2),  #Flow position (x,y) in receiver frame
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
      'Fmvtorcv': mm.Learner('Fmvtorcv',  #Move to receiver
        ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
        ['ps_rcv','p_pour']),
      'Fmvtorcv_rcvmv': mm.Learner('Fmvtorcv_rcvmv',  #Move to receiver: receiver movement
        ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
        ['dps_rcv','v_rcv']),
      'Fmvtopour2': mm.Learner('Fmvtopour2',  #Move to pouring point
        ['ps_rcv','gh_abs','p_pour','p_pour_trg'],
        ['lp_pour']),
      'Fflowc_tip10': mm.Learner('Fflowc_tip10',  #Flow control with tipping.
        ['gh_abs','lp_pour',  #Removed 'p_pour_trg0','p_pour_trg'
        'da_trg','size_srcmouth','material2',
        'dtheta1','dtheta2'],
        ['da_total','lp_flow','flow_var']),  #Removed 'p_pour'
      'Fflowc_shakeA10': mm.Learner('Fflowc_shakeA10',  #Flow control with shake_A.
        ['gh_abs','lp_pour',  #Removed 'p_pour_trg0','p_pour_trg'
        'da_trg','size_srcmouth','material2',
        'dtheta1','shake_spd','shake_axis2'],
        ['da_total','lp_flow','flow_var']),  #Removed 'p_pour'
      'Famount4': mm.Learner('Famount4',  #Amount model common for tip and shake.
        ['lp_pour',  #Removed 'gh_abs','p_pour_trg0','p_pour_trg'
        'da_trg','material2',  #Removed 'size_srcmouth'
        'da_total','lp_flow','flow_var'],
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
      }
    domain.Graph={
      'n0': TDynNode(None,'P1',('Fgrasp','n1')),
      'n1': TDynNode('n0','P2',('Fmvtorcv','n2a'),('Fmvtorcv_rcvmv','n1rcvmv')),
      'n1rcvmv': TDynNode('n1','P1',('Rrcvmv','n1rcvmvr')),
      'n1rcvmvr': TDynNode('n1rcvmv'),
      'n2a': TDynNode('n1','P1',('Fmvtopour2','n2b')),
      'n2b': TDynNode('n2a','P2',('Fnone','n2c'),('Rmvtopour','n2br')),
      'n2br': TDynNode('n2b'),
      'n2c': TDynNode('n2b','Pskill',('Fflowc_tip10','n3ti'),('Fflowc_shakeA10','n3sa')),
      #Tipping:
      'n3ti': TDynNode('n2c','P1',('Famount4','n4ti')),
      'n4ti': TDynNode('n3ti','P1',('Rdamount','n4tir')),
      'n4tir': TDynNode('n4ti'),
      #Shaking-A:
      'n3sa': TDynNode('n2c','P1',('Famount4','n4sa')),
      'n4sa': TDynNode('n3sa','P1',('Rdamount','n4sar')),
      'n4sar': TDynNode('n4sa'),
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

#!/usr/bin/python
from core_tool import *
SmartImportReload('tsim.sm4')
from tsim.sm4 import SetMaterial
SmartImportReload('tsim.dpl_cmn')
from tsim.dpl_cmn import *
def Help():
  return '''Test of DPL for ODE grasping and pouring simulation (ver.2.1).
    Based on tsim.sm4
    Simplified version of tsim2.test_dpl1a
  Usage: tsim2.test_dpl1b'''

def Delta1(dim,s):
  assert(abs(s-int(s))<1.0e-6)
  p= [0.0]*dim
  p[int(s)]= 1.0
  return p

def GetDomain():
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
  domain.Models={
    #key:[In,Out,F],
    'Fnone': [[],[], None],
    'Fgrasp': [['gh_ratio'],['gh_abs'],None],  #Grasping. NOTE: removed ps_rcv
    'Fmvtorcv': [  #Move to receiver
      ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
      ['ps_rcv','p_pour'],None],
    'Fmvtorcv_rcvmv': [  #Move to receiver: receiver movement
      ['ps_rcv','gh_abs','p_pour','p_pour_trg0'],
      ['dps_rcv','v_rcv'],None],
    'Fmvtopour2': [  #Move to pouring point
      ['ps_rcv','gh_abs','p_pour','p_pour_trg'],
      ['lp_pour'],None],
    'Fflowc_tip10': [  #Flow control with tipping.
      ['gh_abs','lp_pour',  #Removed 'p_pour_trg0','p_pour_trg'
       'da_trg','size_srcmouth','material2',
       'dtheta1','dtheta2'],
      ['da_total','lp_flow','flow_var'],None],  #Removed 'p_pour'
    'Fflowc_shakeA10': [  #Flow control with shake_A.
      ['gh_abs','lp_pour',  #Removed 'p_pour_trg0','p_pour_trg'
       'da_trg','size_srcmouth','material2',
       'dtheta1','shake_spd','shake_axis2'],
      ['da_total','lp_flow','flow_var'],None],  #Removed 'p_pour'
    'Famount4': [  #Amount model common for tip and shake.
      ['lp_pour',  #Removed 'gh_abs','p_pour_trg0','p_pour_trg'
       'da_trg','material2',  #Removed 'size_srcmouth'
       'da_total','lp_flow','flow_var'],
      ['da_pour','da_spill2'],None],
    'Rrcvmv':  [['dps_rcv','v_rcv'],[REWARD_KEY],TLocalQuad(13,lambda y:-(np.dot(y[:12],y[:12]) + y[12]*y[12]))],
    'Rmvtopour':  [['p_pour_trg','p_pour'],[REWARD_KEY],TLocalQuad(5,lambda y:-0.1*((y[0]-y[2])**2+(y[1]-y[4])**2))],
    #'Ramount':  [['a_pour','a_trg','a_spill2'],[REWARD_KEY],TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - y[2]*y[2])],
    #'Rdamount':  [['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                  #TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - y[2]*y[2])],
    #'Rdamount':  [['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                  #TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - math.log(1.0+max(0.0,y[2])))],
    #'Rdamount':  [['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                  #TLocalQuad(3,lambda y:-100.0*(y[1]-y[0])*(y[1]-y[0]) - max(0.0,y[2])**2)],
    'Rdamount':  [['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                  TLocalQuad(3,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 1.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2)],
    #'Rdamount':  [['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                  #TLocalQuad(3,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 10.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2)],
    'P1': [[],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])],
    'P2':  [[],[PROB_KEY], TLocalLinear(0,2,lambda x:[1.0]*2,lambda x:[0.0]*2)],
    'Pskill': [['skill'],[PROB_KEY], TLocalLinear(0,2,lambda s:Delta1(2,s[0]),lambda s:[0.0]*2)],
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
  return domain

def Execute(ct):
  sim= ct.sim
  l= ct.sim_local

  # Setup DPL
  l.logdir= '/tmp/dpl/'
  l.opt_conf={
    'model_dir': ct.DataBaseDir()+'models/tsim/v_exp6/',  #'',  Pre-trained w "fixed","fxvs1","random" (for dplD14)
    'model_dir_persistent': False,
    'db_src': '',
    #'db_src': '/tmp/dpl/database.yaml',
    'dpl_options': {
      'opt_log_name': None,  #Not save optimization log.
      },
    }

  domain= GetDomain()

  mm_options= {
    #'type': 'lwr',
    'base_dir': l.logdir+'models/',
    }
  mm= TModelManager(domain.SpaceDefs, domain.Models)
  mm.Load({'options':mm_options})
  if l.opt_conf['model_dir'] not in ('',None):
    if os.path.exists(l.opt_conf['model_dir']+'model_mngr.yaml'):
      mm.Load(LoadYAML(l.opt_conf['model_dir']+'model_mngr.yaml'), l.opt_conf['model_dir'])
    if l.opt_conf['model_dir_persistent']:
      mm.Options['base_dir']= l.opt_conf['model_dir']
    else:
      mm.Options['base_dir']= mm_options['base_dir']
  db= TGraphEpisodeDB()
  if l.opt_conf['db_src'] not in ('',None):
    db.Load(LoadYAML(l.opt_conf['db_src']))

  l.dpl= TGraphDynPlanLearn(domain, db, mm)

  dpl_options={
    'base_dir': l.logdir,
    }
  InsertDict(dpl_options, l.opt_conf['dpl_options'])
  l.dpl.Load({'options':dpl_options})

  l.dpl.MM.Init()
  l.dpl.Init()

  # Done: Setup DPL

  l.dpl.NewEpisode()

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
  l.dpl_res= TContainer()  #l.dpl_res.NODE= res

  ct.srvp.ode_pause()  #Pause during plan/learn
  l.xs.n0= ObserveXSSA(l,None,obs_keys0+('da_trg',))
  pc_rcv= np.array(l.xs.n0['ps_rcv'].X).reshape(4,3).mean(axis=0)  #Center of ps_rcv
  l.xs.n0['p_pour_trg0']= SSA(Vec([-0.3,0.35])+Vec([pc_rcv[0],pc_rcv[2]]))  #A bit above of p_pour_trg
  l.dpl_res.n0= l.dpl.Plan('n0', l.xs.n0)
  ct.srvp.ode_resume()

  gh_ratio= ToList(l.xs.n0['gh_ratio'].X)[0]
  actions['grab']({'gh_ratio':gh_ratio})


  p_pour_trg0= ToList(l.xs.n0['p_pour_trg0'].X)
  actions['move_to_rcv']({'p_pour_trg0':p_pour_trg0})

  p_pour_trg= ToList(l.xs.n0['p_pour_trg'].X)
  actions['move_to_pour']({'p_pour_trg':p_pour_trg})

  selected_skill= ('std_pour','shake_A')[l.xs.n0['skill'].X[0]]

  if selected_skill=='std_pour':
    dtheta1= l.xs.n0['dtheta1'].X[0,0]
    dtheta2= l.xs.n0['dtheta2'].X[0,0]
    actions['std_pour']({'dtheta1':dtheta1, 'dtheta2':dtheta2})

  elif selected_skill=='shake_A':
    dtheta1= l.xs.n0['dtheta1'].X[0,0]
    shake_spd= l.xs.n0['shake_spd'].X[0,0]
    shake_axis2= ToList(l.xs.n0['shake_axis2'].X)
    actions['shake_A']({'dtheta1':dtheta1, 'shake_spd':shake_spd, 'shake_axis2':shake_axis2})

  l.dpl.EndEpisode()


def ConfigCallback(ct,l,sim):
  SetMaterial(l, preset=('bounce','nobounce','natto','ketchup')[RandI(4)])
  l.config.SrcSize2H= Rand(0.02,0.09)  #Mouth size of source container
  l.config.RcvPos= [0.8+0.6*(random.random()-0.5), l.config.RcvPos[1], l.config.RcvPos[2]]

def Run(ct,*args):
  l= TContainer(debug=True)
  l.config_callback= ConfigCallback
  ct.Run('tsim2.setup', l)
  sim= ct.sim
  l= ct.sim_local

  try:
    Execute(ct)

  finally:
    sim.StopPubSub(ct,l)
    l.sensor_callback= None
    ct.srvp.ode_pause()

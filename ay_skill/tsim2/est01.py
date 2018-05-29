#!/usr/bin/python
from core_tool import *
SmartImportReload('tsim.dpl_cmn')
from tsim.dpl_cmn import *
def Help():
  return '''State estimation --- initial attempt.
    This script gathers data.
    Based on tsim2.dplD22.
  Usage: tsim2.est01'''

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

def GetDomain(l):
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
    'Tlp_flow':  #Transform from ps_rcv,p_pour,lpp_flow to lp_flow2
      mm.Manual(['ps_rcv','p_pour','lpp_flow'],['lp_flow2'],
        TLocalLinear(17,2,FdF=Tlp_flow_dF)),
    'Famount5': mm.Learner('Famount5',  #Amount model common for tip and shake.
      ['da_total','lp_flow2','flow_var'],
      ['da_pour','da_spill2']),
    'Rrcvmv':  mm.Manual(['dps_rcv','v_rcv'],[REWARD_KEY],TLocalQuad(13,lambda y:-(np.dot(y[:12],y[:12]) + y[12]*y[12]))),
    'Rmvtopour':  mm.Manual(['p_pour_trg','p_pour'],[REWARD_KEY],TLocalQuad(5,lambda y:-0.1*((y[0]-y[2])**2+(y[1]-y[4])**2))),
    'Rdamount':  mm.Manual(['da_pour','da_trg','da_spill2'],[REWARD_KEY],
                  TLocalQuad(3,lambda y:-100.0*max(0.0,y[1]-y[0])**2 - 1.0*max(0.0,y[0]-y[1])**2 - 1.0*max(0.0,y[2])**2)),
    'P1': mm.Manual([],[PROB_KEY], TLocalLinear(0,1,lambda x:[1.0],lambda x:[0.0])),
    'P2':  mm.Manual([],[PROB_KEY], TLocalLinear(0,2,lambda x:[1.0]*2,lambda x:[0.0]*2)),
    'Pskill': mm.Manual(['skill'],[PROB_KEY], TLocalLinear(0,2,lambda s:Delta1(2,s[0]),lambda s:[0.0]*2)),
    }
  domain.Graph={
    #n0->'Fgrasp'->n1->'Fmvtorcv2'->n2->'Fnone'->n3->'Fmvtopour3' A/
    #                               n2->'Rrcvmv'->n2r
    #/A->n4->'Tp_pour_z'->n5 B/
    #    n4->'Rmvtopour'->n4r
    #/B->'Fflowc_tip11'   ->n6ti->'Tlp_flow'->n7ti->'Famount5'->n8ti->'Rdamount'->n8tir
    #/B->'Fflowc_shakeA11'->n6sa->'Tlp_flow'->n7sa->'Famount5'->n8sa->'Rdamount'->n8sar
    'n0': TDynNode(None,'P1',('Fgrasp','n1')),
    'n1': TDynNode('n0','P1',('Fmvtorcv2','n2')),
    'n2': TDynNode('n1','P2',('Fnone','n3'),('Rrcvmv','n2r')),
    'n2r': TDynNode('n2'),
    'n3': TDynNode('n2','P1',('Fmvtopour3','n4')),
    'n4': TDynNode('n3','P2',('Tp_pour_z','n5'),('Rmvtopour','n4r')),
    'n4r': TDynNode('n4'),
    'n5': TDynNode('n4','Pskill',('Fflowc_tip11','n6ti'),('Fflowc_shakeA11','n6sa')),
    #Tipping:
    'n6ti': TDynNode('n5','P1',('Tlp_flow','n7ti')),
    'n7ti': TDynNode('n6ti','P1',('Famount5','n8ti')),
    'n8ti': TDynNode('n7ti','P1',('Rdamount','n8tir')),
    'n8tir': TDynNode('n8ti'),
    #Shaking-A:
    'n6sa': TDynNode('n5','P1',('Tlp_flow','n7sa')),
    'n7sa': TDynNode('n6sa','P1',('Famount5','n8sa')),
    'n8sa': TDynNode('n7sa','P1',('Rdamount','n8sar')),
    'n8sar': TDynNode('n8sa'),
    }

  return domain, mm


def Execute(ct,l,observe_hidden_st):
  ct.Run('tsim2.setup', l)
  sim= ct.sim
  hidden_st= observe_hidden_st(l)
  #l= ct.sim_local

  actions={
    'grab'         : lambda a: ct.Run('tsim2.act.grab', a),
    'move_to_rcv'  : lambda a: ct.Run('tsim2.act.move_to_rcv', a),
    'move_to_pour' : lambda a: ct.Run('tsim2.act.move_to_pour', a),
    'std_pour'     : lambda a: ct.Run('tsim2.act.std_pour', a),
    'shake_A'      : lambda a: ct.Run('tsim2.act.shake_A', a),
    }

  #NOTE: Do not include 'da_trg' in obs_keys0 since 'da_trg' should be kept during some node transitions.
  #NOTE: Removed 'material2' from obs_keys0 since it is assumed as a hidden variable.
  obs_keys0= ('ps_rcv','p_pour','p_pour_z','lp_pour','a_trg','size_srcmouth')
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
    for key,value in hidden_st.iteritems():
      l.xs.n0[key]= value
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
    #l.dpl.UpdateModel('Fgrasp',l.xs.prev,l.xs.n1, not_learn=l.not_learn)
    ##res= l.dpl.Plan('n1', l.xs.n1)
    l.idb.n1= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n1',xs=l.xs.n1)
    l.xs.prev= l.xs.n1
    l.idb.prev= l.idb.n1

  p_pour_trg0= ToList(l.xs.n1['p_pour_trg0'].X)
  p_pour_trg= ToList(l.xs.n1['p_pour_trg'].X)
  actions['move_to_rcv']({'p_pour_trg0':p_pour_trg0})
  VizPP(l,[p_pour_trg0[0],0.0,p_pour_trg0[1]],[0.,1.,0.])
  VizPP(l,[p_pour_trg[0],0.0,p_pour_trg[1]],[0.5,0.,1.])

  with sim.TPause(ct):  #Pause during plan/learn
    CPrint(2,'Node:','n2')
    l.xs.n2= CopyXSSA(l.xs.prev)
    InsertDict(l.xs.n2, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab+('dps_rcv','v_rcv')))
    #l.dpl.UpdateModel('Fmvtorcv2',l.xs.prev,l.xs.n2, not_learn=l.not_learn)
    ##res= l.dpl.Plan('n2', l.xs.n2)
    l.idb.n2= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n2',xs=l.xs.n2)

    #Branch-1: main procedure
    #Fnone: just go to n3

    #Branch-2: reward
    CPrint(2,'Node:','n2r')
    #Since we have 'Rrcvmv', we just use it to get the next XSSA
    l.xs.n2r= l.dpl.Forward('Rrcvmv',l.xs.n2)
    l.idb.n2r= l.dpl.DB.AddToSeq(parent=l.idb.n2,name='n2r',xs=l.xs.n2r)

  repeated= False  #For try-and-error learning
  while True:  #Try-and-error starts from here.
    #Three cases of parent of l.idb.n3: l.idb.n2, l.idb.n8ti, l.idb.n8sa

    with sim.TPause(ct):  #Pause during plan/learn
      #Plan l.p_pour_trg
      CPrint(2,'Node:','n3')
      l.xs.n3= CopyXSSA(l.xs.prev)
      if repeated:
        #Delete actions and selections (e.g. skill) to plan again from initial guess.
        for key in l.xs.n3.keys():
          if l.dpl.d.SpaceDefs[key].Type in ('action','select'):
            del l.xs.n3[key]
      InsertDict(l.xs.n3, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab+('da_trg',)))
      ##TEST: Heuristic init guess
      ##l.xs.n3['skill']= SSA([1])
      #res= l.dpl.Plan('n3', l.xs.n3)
      l.idb.n3= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n3',xs=l.xs.n3)
      l.xs.prev= l.xs.n3
      l.idb.prev= l.idb.n3

    p_pour_trg= ToList(l.xs.n3['p_pour_trg'].X)
    actions['move_to_pour']({'p_pour_trg':p_pour_trg})
    l.user_viz.pop()
    VizPP(l,[p_pour_trg[0],0.0,p_pour_trg[1]],[1.,0.,1.])


    with sim.TPause(ct):  #Pause during plan/learn
      CPrint(2,'Node:','n4')
      l.xs.n4= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.n4, ObserveXSSA(l,l.xs.prev,obs_keys_after_grab))
      #l.dpl.UpdateModel('Fmvtopour3',l.xs.prev,l.xs.n4, not_learn=l.not_learn)
      ##res= l.dpl.Plan('n4', l.xs.n4)
      l.idb.n4= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n4',xs=l.xs.n4)
      l.xs.prev= l.xs.n4
      l.idb.prev= l.idb.n4

    #Branch-1: main procedure
    #Just go to 'n5'

    #Branch-2: reward
    CPrint(2,'Node:','n4r')
    #Since we have 'Rmvtopour', we just use it to get the next XSSA
    l.xs.n4r= l.dpl.Forward('Rmvtopour',l.xs.prev)
    l.idb.n4r= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n4r',xs=l.xs.n4r)

    with sim.TPause(ct):  #Pause during plan/learn
      #Plan l.selected_skill from ('std_pour','shake_A','shake_B')
      CPrint(2,'Node:','n5')
      l.xs.n5= CopyXSSA(l.xs.prev)
      InsertDict(l.xs.n5, ObserveXSSA(l,l.xs.prev,obs_keys_before_flow))
      ##l.dpl.UpdateModel('Tp_pour_z',l.xs.prev,l.xs.n5, not_learn=l.not_learn)
      ##res= l.dpl.Plan('n5', l.xs.n5)
      l.idb.n5= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n5',xs=l.xs.n5)
      l.xs.prev= l.xs.n5
      l.idb.prev= l.idb.n5

    selected_skill= ('std_pour','shake_A')[l.xs.n5['skill'].X[0]]
    #selected_skill= 'shake_A'
    if selected_skill=='std_pour':
      dtheta1= l.xs.n5['dtheta1'].X[0,0]
      dtheta2= l.xs.n5['dtheta2'].X[0,0]
      actions['std_pour']({'dtheta1':dtheta1, 'dtheta2':dtheta2})

      with sim.TPause(ct):  #Pause during plan/learn
        CPrint(2,'Node:','n6ti')
        l.xs.n6ti= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n6ti, ObserveXSSA(l,l.xs.prev,obs_keys_after_flow))
        #l.dpl.UpdateModel('Fflowc_tip11',l.xs.prev,l.xs.n6ti, not_learn=l.not_learn)
        ##res= l.dpl.Plan('n6ti', l.xs.n6ti)
        l.idb.n6ti= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n6ti',xs=l.xs.n6ti)
        l.xs.prev= l.xs.n6ti
        l.idb.prev= l.idb.n6ti

        #Transform
        CPrint(2,'Node:','n7ti')
        l.xs.n7ti= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n7ti, ObserveXSSA(l,l.xs.prev,()))  #Observation is omitted since there is no change
        ##l.dpl.UpdateModel('Tlp_flow',l.xs.prev,l.xs.n7ti, not_learn=l.not_learn)
        ##res= l.dpl.Plan('n7ti', l.xs.n7ti)
        l.idb.n7ti= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n7ti',xs=l.xs.n7ti)
        l.xs.prev= l.xs.n7ti
        l.idb.prev= l.idb.n7ti

        CPrint(2,'Node:','n8ti')
        l.xs.n8ti= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n8ti, ObserveXSSA(l,l.xs.prev,()))  #Observation is omitted since there is no change
        #l.dpl.UpdateModel('Famount5',l.xs.prev,l.xs.n8ti, not_learn=l.not_learn)
        ##l.dpl.UpdateModel('Famount5',xs_in,l.xs.n8ti, not_learn=l.not_learn)
        ##res= l.dpl.Plan('n8ti', l.xs.n8ti)
        l.idb.n8ti= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n8ti',xs=l.xs.n8ti)
        l.xs.prev= l.xs.n8ti
        l.idb.prev= l.idb.n8ti

        CPrint(2,'Node:','n8tir')
        l.xs.n8tir= l.dpl.Forward('Rdamount',l.xs.prev)
        l.idb.n8tir= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n8tir',xs=l.xs.n8tir)

    elif selected_skill=='shake_A':
      dtheta1= l.xs.n5['dtheta1'].X[0,0]
      shake_spd= l.xs.n5['shake_spd'].X[0,0]
      shake_axis2= ToList(l.xs.n5['shake_axis2'].X)
      actions['shake_A']({'dtheta1':dtheta1, 'shake_spd':shake_spd, 'shake_axis2':shake_axis2})

      with sim.TPause(ct):  #Pause during plan/learn
        CPrint(2,'Node:','n6sa')
        l.xs.n6sa= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n6sa, ObserveXSSA(l,l.xs.prev,obs_keys_after_flow))
        #l.dpl.UpdateModel('Fflowc_shakeA11',l.xs.prev,l.xs.n6sa, not_learn=l.not_learn)
        ##res= l.dpl.Plan('n6sa', l.xs.n6sa)
        l.idb.n6sa= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n6sa',xs=l.xs.n6sa)
        l.xs.prev= l.xs.n6sa
        l.idb.prev= l.idb.n6sa

        #Transform
        CPrint(2,'Node:','n7sa')
        l.xs.n7sa= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n7sa, ObserveXSSA(l,l.xs.prev,()))  #Observation is omitted since there is no change
        ##l.dpl.UpdateModel('Tlp_flow',l.xs.prev,l.xs.n7sa, not_learn=l.not_learn)
        ##res= l.dpl.Plan('n7sa', l.xs.n7sa)
        l.idb.n7sa= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n7sa',xs=l.xs.n7sa)
        l.xs.prev= l.xs.n7sa
        l.idb.prev= l.idb.n7sa

        CPrint(2,'Node:','n8sa')
        l.xs.n8sa= CopyXSSA(l.xs.prev)
        InsertDict(l.xs.n8sa, ObserveXSSA(l,l.xs.prev,()))  #Observation is omitted since there is no change
        #l.dpl.UpdateModel('Famount5',l.xs.prev,l.xs.n8sa, not_learn=l.not_learn)
        ##l.dpl.UpdateModel('Famount5',xs_in,l.xs.n8sa, not_learn=l.not_learn)
        ##res= l.dpl.Plan('n8sa', l.xs.n8sa)
        l.idb.n8sa= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n8sa',xs=l.xs.n8sa)
        l.xs.prev= l.xs.n8sa
        l.idb.prev= l.idb.n8sa

        CPrint(2,'Node:','n8sar')
        l.xs.n8sar= l.dpl.Forward('Rdamount',l.xs.prev)
        l.idb.n8sar= l.dpl.DB.AddToSeq(parent=l.idb.prev,name='n8sar',xs=l.xs.n8sar)

    ## Conditions to break the try-and-error loop
    #if l.IsPoured():
      #break
    #if l.IsTimeout() or l.IsEmpty():  # or l.IsSpilled()
      #break
    #if not IsSuccess(l.exec_status):
      #break
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
  elif l.mtr_smsz=='rand_smsz':
    l.config.SrcSize2H= Rand(0.02,0.09)  #Mouth size of source container
  CPrint(3,'l.config.ViscosityParam1=',l.config.ViscosityParam1)
  CPrint(3,'l.config.SrcSize2H=',l.config.SrcSize2H)

  config= {key: getattr(l.config,key) for key in l.config.__slots__}
  l.config_log.append(config)

def GetHiddenSt(l):
  #xs[key]= SSA([l.config.ContactBounce, l.config.ContactBounceVel,
                #l.config.ViscosityParam1*1.0e6, l.config.ViscosityMaxDist])
  #hidden_st= {'material2': [l.config.ContactBounce, l.config.ContactBounceVel,
                            #l.config.ViscosityParam1*1.0e6, l.config.ViscosityMaxDist]}
  hidden_st= {
    'material2': SSA([0.0, 0.0, 0.0, 0.0]),
    'material2_true': SSA([l.config.ContactBounce, l.config.ContactBounceVel,
                           l.config.ViscosityParam1*1.0e6, l.config.ViscosityMaxDist]),  #True value for log.
    }
  for key,value in l.opt_conf['hidden_st'].iteritems():
    hidden_st[key]= value
  if 'hiddenst_callback' in l.opt_conf and l.opt_conf['hiddenst_callback']!=None:
    l.opt_conf['hiddenst_callback'](l,hidden_st)
  return hidden_st

#Wrapping tsim2.setup.SetMaterial == tsim.sm1.SetMaterial
def SetMaterial(ct, preset=None, kind=None, bounce1=0.1, bounce2=0.2, viscous1=2.5e-7, viscous2=0.2):
  l= TContainer()
  l.config= TContainer()
  m_setup= ct.Load('tsim2.setup')
  m_setup.SetMaterial(l, preset, kind, bounce1, bounce2, viscous1, viscous2)
  return {key:getattr(l.config,key) for key in ('ContactBounce','ContactBounceVel','ViscosityParam1', 'ViscosityMaxDist')}


def Run(ct,*args):
  l= TContainer(debug=True)
  l.logdir= args[0] if len(args)>0 else '/tmp/dpl03/'
  opt_conf= args[1] if len(args)>1 else None
  #l.planlearn_callback= PlanLearnCallback
  l.config_callback= ConfigCallback
  #l.m_sm= ct.Load('tsim.sm4')
  if opt_conf is None:
    #opt_conf={}
    #Setup for experiments:
    opt_conf={
      'mtr_smsz': 'rand_smsz',  #'fixed', 'fxvs1', 'random', 'viscous'
      'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
      'model_dir': ct.DataBaseDir()+'models/tsim2/v2/',  #'',  Trained with 42 episodes of ('random',None) with dplD23 setup (extended graph)
      'model_dir_persistent': False,
      'num_log_interval': 1,
      'config': {
        #'BallType': 1, 'MaxContacts':1,
        },
      }
    #Setting common material among episodes:
    #presets= ('bounce','nobounce','natto','ketchup')
    presets= ('natto','ketchup')
    for key,value in SetMaterial(ct, preset=presets[RandI(len(presets))]).iteritems():
      opt_conf['config'][key]= value
    ##{{{Replay:
    #l.logdir= '/tmp/dpl03b/'
    #opt_conf['rcv_size']= None
    #opt_conf['mtr_smsz']= None
    #opt_conf['config']= LoadYAML('/tmp/dpl03/config_log.yaml')[0]
    ##  [[[Replaying, but random receiver position and source mouth size:
    #del opt_conf['config']['RcvPos']
    #opt_conf['mtr_smsz']= 'rand_smsz'
    ##  ]]]
    #opt_conf['hidden_st']= {'material2': SSA([0.5420153917947872, 0.0, 0.6119022910507373, 0.43969985569013126])}
    ##hidden_st= {SSA('material2': [l.config.ContactBounce, l.config.ContactBounceVel,
                                  ##l.config.ViscosityParam1*1.0e6, l.config.ViscosityMaxDist])}
    ##}}}

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
    'hidden_st': {},
    'hiddenst_callback': None,
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
    assert(l.rwd_schedule is None)

  def LogDPL(l):
    ##SaveYAML(l.dpl.MM.Save(l.dpl.MM.Options['base_dir']), l.dpl.MM.Options['base_dir']+'model_mngr.yaml')
    #l.mm.Save()
    SaveYAML(l.dpl.DB.Save(), l.logdir+'database.yaml', interactive=False)
    SaveYAML(l.dpl.Save(), l.logdir+'dpl.yaml', interactive=False)
    SaveYAML(l.config_log, l.logdir+'config_log.yaml', interactive=False)


  l.config_log= []

  if l.interactive and 'log_dpl' in ct.__dict__ and (CPrint(1,'Restart from existing DPL?'), AskYesNo())[1]:
    l.dpl= ct.log_dpl
    l.mm= ct.log_mm
    l.restarting= True
  else:
    #Setup dynamic planner/learner
    domain,mm= GetDomain(l)

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
        Execute(ct,l,GetHiddenSt)
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

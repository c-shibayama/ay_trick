#!/usr/bin/python
from core_tool import *
SmartImportReload('lib.tsim2.est01')
from lib.tsim2.est01 import (
  Delta1,
  Tlp_pour_dF,
  Tlp_flow_dF,
  GetDomain,
  )

def Help():
  return '''State estimation --- initial attempt.
    This script estimates hidden state (material2) from data
    gathered by tsim2.est01.
    Based on tsim2.est01.
    This script uses single data.
  Usage: tsim2.est01a'''

def Estimate(ct,*args):
  l= TContainer(debug=True)
  l.logdir= args[0] if len(args)>0 else '/tmp/dpl03a/'
  opt_conf= args[1] if len(args)>1 else None
  if opt_conf is None:
    #opt_conf={}
    opt_conf={
      'hidden_st0': {'material2':SSA([0.0]*4),},  #Initial value of hidden states.
      'db_data': '/tmp/dpl03/database.yaml',
      }

  l.opt_conf={
    'hidden_st0': {},  #Initial value of hidden states.
    'model_dir': ct.DataBaseDir()+'models/tsim2/v2/',  #'',  Trained with 42 episodes of ('random',None) with dplD23 setup (extended graph)
    'model_dir_persistent': False,  #If False, models are saved in l.logdir, i.e. different one from 'model_dir'
    'db_src': '',
    'dpl_options': {
      'opt_log_name': None,  #Not save optimization log.
      },
    }
  InsertDict(l.opt_conf, opt_conf)

  domain,mm= GetDomain(l)

  SP= TCompSpaceDef
  #Changing material2 to action to optimize.
  domain.SpaceDefs['material2']= SP('action',4,min=[0.0,0.0,0.0,0.0],max=[1.0,0.5,10.0,0.5])  #Material property (e.g. viscosity)
  #Changing actions to state to avoid optimizing them.
  domain.SpaceDefs['skill']= SP('state',1)  #Skill selection
  domain.SpaceDefs['gh_ratio']= SP('state',1)
  domain.SpaceDefs['p_pour_trg']= SP('state',2)
  domain.SpaceDefs['dtheta1']= SP('state',1)
  domain.SpaceDefs['dtheta2']= SP('state',1)
  domain.SpaceDefs['shake_spd']= SP('state',1)
  domain.SpaceDefs['shake_axis2']= SP('state',2)
  #Adding observations:
  for key in ['da_total','lpp_flow','flow_var']:
    domain.SpaceDefs[key+'_true']= domain.SpaceDefs[key]

  #key:[In,Out,F]
  #cf. Output of Fflowc_*: ['da_total','lpp_flow','flow_var']
  domain.Models[
    'Restflowc']= mm.Manual(['da_total','lpp_flow','flow_var', 'da_total_true','lpp_flow_true','flow_var_true'],
                            [REWARD_KEY],TLocalQuad(8,lambda y:-(np.dot(Vec(y[4:])-y[:4],Vec(y[4:])-y[:4]) )))

  #Modify Graph for state estimation:
  domain.Graph={
    #n5->'Fflowc_tip11'   ->n6ti->'Restflowc'->n7ti
    #n5->'Fflowc_shakeA11'->n6sa->'Restflowc'->n7sa
    'n5': TDynNode(None,'Pskill',('Fflowc_tip11','n6ti'),('Fflowc_shakeA11','n6sa')),
    #Tipping:
    'n6ti': TDynNode('n5','P1',('Restflowc','n7ti')),
    'n7ti': TDynNode('n6ti'),
    #Shaking-A:
    'n6sa': TDynNode('n5','P1',('Restflowc','n7sa')),
    'n7sa': TDynNode('n6sa'),
    }

  #obs_keys0= ('ps_rcv','p_pour','p_pour_z','lp_pour','a_trg','size_srcmouth',
              #'da_trg',
              #'skill','gh_ratio','p_pour_trg','p_pour_trg0','dtheta1','dtheta2','shake_spd','shake_axis2',)
  '''cf. Input of
  'Fflowc_tip11':
      ['gh_abs','p_pour_z',
      'da_trg','size_srcmouth','material2',
      'dtheta1','dtheta2'],
  'Fflowc_shakeA11':
      ['gh_abs','p_pour_z',
      'da_trg','size_srcmouth','material2',
      'dtheta1','shake_spd','shake_axis2'], '''
  input_keys= ('gh_abs','p_pour_z',
               'da_trg','size_srcmouth',
               'dtheta1','dtheta2','dtheta1','shake_spd','shake_axis2',
               'skill',)

  db_data= TGraphEpisodeDB()
  db_data.Load(LoadYAML(l.opt_conf['db_data']))
  #print db_data.Entry[0].Seq[0].XS['p_pour_z']


  l.dpl= TGraphDynPlanLearn2(domain)
  l.mm= mm
  l.restarting= False

  dpl_options={
    'base_dir': l.logdir,
    }
  InsertDict(dpl_options, l.opt_conf['dpl_options'])
  l.dpl.Load({'options':dpl_options})


  l.xs= TContainer()  #l.xs.NODE= XSSA
  l.idb= TContainer()  #l.idb.NODE= index in DB

  #Copy features from data:
  #xs_n5= db_data.Entry[0].Seq[0].XS
  #l.xs.n5= {key:xs_n5[key] for key in obs_keys0}
  xs_n5= next(node for node in db_data.Entry[0].Seq if node.Name=='n5').XS
  l.xs.n5= {key:xs_n5[key] for key in input_keys}

  #Initializing hidden variables (actions).
  for key,value in l.opt_conf['hidden_st0'].iteritems():
    l.xs.n5[key]= value

  xs_nx= next(node for node in db_data.Entry[0].Seq if node.Name in ('n6ti','n6sa')).XS
  for key in ['da_total','lpp_flow','flow_var']:
    l.xs.n5[key+'_true']= xs_nx[key]

  res= l.dpl.Plan('n5', l.xs.n5)

  print 'Setup:'
  print l.opt_conf
  print 'XSSA after optimization:'
  print l.xs.n5
  print 'material2=',l.xs.n5['material2']

  return {key:value for key,value in l.xs.n5.iteritems() if domain.SpaceDefs[key].Type!='state'}


def Run(ct,*args):
  return Estimate(ct,*args)

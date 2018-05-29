#!/usr/bin/python
from core_tool import *
SmartImportReload('tsim2.est01')
from tsim2.est01 import (
  Delta1,
  Tlp_pour_dF,
  Tlp_flow_dF,
  GetDomain,
  )

def Help():
  return '''State estimation --- version 2.
    This script estimates hidden state (material2) from data
    gathered by tsim2.est01.
    Based on tsim2.est01a.
    This script uses multiple data (material2 is common (same value) among the samples).
  Usage: tsim2.est01b'''

'''A generator function y=F().  There is no input.
This function class selects an XSSA from an array of XSSA.
i.e. it returns A[i].'''
class TArraySelector(TFunctionApprox):
  #Number of x-dimensions
  @property
  def Dx(self):
    return 0

  #Number of y-dimensions
  @property
  def Dy(self):
    return self.dy

  def __init__(self, dy, i, A=None):
    TFunctionApprox.__init__(self)
    self.i= i
    self.dy= dy
    self.A= A

  #Whether prediction is available (False if the model is not learned).
  def IsPredictable(self):
    return True  #This class does not learn anything.

  '''
  Do prediction.
    Return a TPredRes instance.
    x_var: Covariance of x.  If a scholar is given, we use diag(x_var,x_var,..).
    with_var: Whether compute a covariance matrix of error at the query point as well.
    with_grad: Whether compute a gradient at the query point as well.
  '''
  def Predict(self, x, x_var=0.0, with_var=False, with_grad=False):
    #x_var, var_is_zero= RegularizeCov(x_var, len(x))
    ys= self.A[self.i]
    dy= np.zeros((0,len(ys.X)))
    var,_= RegularizeCov(ys.Cov, len(ys.X))
    res= self.TPredRes()
    res.Y= ys.X
    if with_var:  res.Var= var
    if with_grad:  res.Grad= dy
    return res


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
    ##n0->{'G_0'->n5_0, 'G_1'->n5_1, ...}
    ##n5_{i}->'Fflowc_tip11'   ->n6ti_{i}->'Restflowc'->n7ti_{i}
    ##n5_{i}->'Fflowc_shakeA11'->n6sa_{i}->'Restflowc'->n7sa_{i}
    #'n5': TDynNode(None,'Pskill',('Fflowc_tip11','n6ti'),('Fflowc_shakeA11','n6sa')),
    ##Tipping:
    #'n6ti': TDynNode('n5','P1',('Restflowc','n7ti')),
    #'n7ti': TDynNode('n6ti'),
    ##Shaking-A:
    #'n6sa': TDynNode('n5','P1',('Restflowc','n7sa')),
    #'n7sa': TDynNode('n6sa'),
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
  input_keys= ['gh_abs','p_pour_z',
               'da_trg','size_srcmouth',
               'dtheta1','dtheta2','dtheta1','shake_spd','shake_axis2',
               'skill',]
  output_keys= ['da_total','lpp_flow','flow_var']
  output_true_keys= [key+'_true' for key in output_keys]

  db_data= TGraphEpisodeDB()
  db_data.Load(LoadYAML(l.opt_conf['db_data']))
  #print db_data.Entry[0].Seq[0].XS['p_pour_z']

  #Generate graph and models according to the data:
  N= len(db_data.Entry)  #Number of samples
  xs_n5_array= [None]*N
  domain.Models['PmN']= [[],[PROB_KEY],
                         TLocalLinear(0,N,lambda x:[1.0]*N,lambda x:[0.0]*N)]
  domain.Graph['n0']= TDynNode(None,'PmN',*(('G_%d'%i,'n5_%d'%i) for i in range(N)))
  for i in range(N):
    domain.Models['G_%d'%i]= [[],input_keys+output_true_keys,TArraySelector(N,i,xs_n5_array)]
    domain.Graph['n5_%d'%i]= TDynNode(None,'Pskill',('Fflowc_tip11','n6ti_%d'%i),('Fflowc_shakeA11','n6sa_%d'%i))
    #Tipping:
    domain.Graph['n6ti_%d'%i]= TDynNode('n5_%d'%i,'P1',('Restflowc','n7ti_%d'%i))
    domain.Graph['n7ti_%d'%i]= TDynNode('n6ti_%d'%i)
    #Shaking-A:
    domain.Graph['n6sa_%d'%i]= TDynNode('n5_%d'%i,'P1',('Restflowc','n7sa_%d'%i))
    domain.Graph['n7sa_%d'%i]= TDynNode('n6sa_%d'%i)

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

  l.xs.n0= {}

  #Copy features from data:
  for i in range(N):
    xs= {}

    xs_n5= next(node for node in db_data.Entry[i].Seq if node.Name=='n5').XS
    for key in input_keys:  xs[key]= xs_n5[key]
    xs_nx= next(node for node in db_data.Entry[i].Seq if node.Name in ('n6ti','n6sa')).XS
    for key in output_keys:  xs[key+'_true']= xs_nx[key]

    x,cov,dims= SerializeXSSA(domain.SpaceDefs, xs, input_keys+output_true_keys)
    xs_n5_array[i]= SSA(x,cov)

  #Initializing hidden variables (actions).
  for key,value in l.opt_conf['hidden_st0'].iteritems():
    l.xs.n0[key]= value

  res= l.dpl.Plan('n0', l.xs.n0)

  print 'Setup:'
  print l.opt_conf
  print 'XSSA after optimization:'
  print l.xs.n0
  print 'material2=',l.xs.n0['material2']

  return {key:value for key,value in l.xs.n0.iteritems() if domain.SpaceDefs[key].Type!='state'}


def Run(ct,*args):
  return Estimate(ct,*args)

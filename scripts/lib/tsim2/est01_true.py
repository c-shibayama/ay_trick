#!/usr/bin/python
from core_tool import *
SmartImportReload('lib.tsim2.est01')
from lib.tsim2.est01 import (
  SetMaterial,
  )
from lib.tsim2.est01 import Run as ExecutePouring

def Help():
  return '''State estimation --- planning with true hidden parameters.
    This script executes tsim2.est01 with assigning true values to the hidden parameters.
  Usage: tsim2.est01_true'''

def HiddenstCallback(ct,est01_l,hidden_st):
  for key in ('material2',):
    hidden_st[key]= hidden_st[key+'_true']

def Run(ct,*args):
  l= TContainer(debug=True)
  l.logdir= args[0] if len(args)>0 else '/tmp/dpl03/'
  opt_conf= args[1] if len(args)>1 else None
  if opt_conf is None:
    opt_conf={}
    #opt_conf={
      #'config': {
        #'BallType': 1, 'MaxContacts':1, 'BallRad': 0.032,
        #},
      #}

  l.opt_conf={
      'mtr_smsz': 'rand_smsz',  #'fixed', 'fxvs1', 'random', 'viscous'
      'mtr_presets': ('bounce','nobounce','natto','ketchup'),  #Presets of material (one of presents is chosen randomly).
      'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
      'num_log_interval': 1,
      #'num_episodes': 5,
      #'interactive': False,
      'config': {},
      'hiddenst_callback': lambda l,hidden_st,ct=ct: HiddenstCallback(ct,l,hidden_st),
    }
  InsertDict(l.opt_conf, opt_conf)

  #Setting common material property among episodes from presets:
  presets= l.opt_conf['mtr_presets']
  if presets is not None and len(presets)>0:
    #presets= ('natto','ketchup')
    for key,value in SetMaterial(ct, preset=presets[RandI(len(presets))]).iteritems():
      l.opt_conf['config'][key]= value

  #Make l.logdir+'database.yaml' blank to reset data for estimating hidden state.
  OpenW(l.logdir+'database.yaml').close()

  #ct.Run('tsim2.est01',l.logdir,l.opt_conf)
  ExecutePouring(ct,l.logdir,l.opt_conf)


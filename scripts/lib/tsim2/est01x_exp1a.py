#!/usr/bin/python
from core_tool import *
from ay_py.tool.exp import Exp
def Help():
  return '''Script of experiments.
  Usage: tsim2.est01x_exp1a'''

def Run(ct,*args):
  main_script= 'tsim2.est01_true'
  #basedir='{home}/data/tsim2est01/EST01_True_2017_06_19/'
  basedir='{home}/data/tsim2est01b/EST01_True_2017_07_12/'
  logdir='{base}/{cond}/run{n:03}/'
  dirs= ['seq','act','sm','models','models/train']
  num_runs= 100
  common= {
    'num_episodes': 1,
    'interactive': False,
    'mtr_smsz': 'rand_smsz',  #'fixed', 'fxvs1', 'random', 'viscous'
    'mtr_presets': ('bounce','nobounce','natto','ketchup'),  #Presets of material (one of presents is chosen randomly).
    'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
    'num_log_interval': 1,
    'config': {},
    }
  ##test:
  #basedir='/tmp/tsim2est01/EST01_Eb_2017_06_19/'
  #num_runs= 1
  #common['num_episodes']= 1

  compared_options= {
    'True_Same': {
      'config': {
        #'BallType': 1, 'MaxContacts':1,
        },
      },
    'True_MtrShape': {
      'config': {
        'BallType': 1, 'MaxContacts':1, 'BallRad': 0.032,
        },
      },
    }  #compared_options

  conditions= {cond:copy.deepcopy(common) for cond in compared_options.keys()}
  for cond,options in compared_options.iteritems():
    InsertDict(conditions[cond], options)

  main= lambda logdir,options: ct.Run(main_script,logdir,options)
  Exp(main, basedir, logdir, dirs, num_runs, conditions)


#!/usr/bin/python
from core_tool import *
def Help():
  return '''Replay tsim.dplD14 for taking a video, etc.
  Usage: tsim.dplD14replay DATADIR
    e.g. tsim.dplD14replay '/home/akihiko/data/dplexp06/DPLd_2016_03_11/16_rnd_ns_ve5/run008/'
  '''

def Run(ct,*args):
  datadir= args[0] if len(args)>0 else '/tmp/dpl/'

  logdir= '/tmp/dpl/'
  opt_conf={
    'interactive': True,
    'not_learn': True,  #Models are not trained.
    'num_episodes': 10,
    'num_log_interval': 100,
    'rcv_size': 'static',  #'static', 'random'
    'mtr_smsz': 'random',  #'fixed', 'fxvs1', 'random', 'viscous'
    'rwd_schedule': None,  #None, 'early_tip', 'early_shakeA'
    'model_dir': datadir+'models/',  #'',
    'model_dir_persistent': False,  #If False, models are saved in l.logdir, i.e. different one from 'model_dir'
    'db_src': datadir+'database.yaml',
    'config': {},  #Config of the simulator
    'dpl_options': {
      'opt_log_name': None,  #Not save optimization log.
      },
    }
  ct.Run('tsim.dplD14',logdir,opt_conf)


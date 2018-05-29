#!/usr/bin/python
from core_tool import *
SmartImportReload('tsim.dpl_cmn')
from tsim.dpl_cmn import *
def Help():
  return '''Test of using actions for ODE grasping and pouring simulation (ver.2.1).
    Replaying an episode to check the simulation consistency.
    Based on tsim2.test_act
  Usage: tsim2.test_replay'''

def TestConfigCallback(ct,l,sim):
  for key,value in l.opt_conf['config'].iteritems():
    setattr(l.config, key, value)

  #config= {key: getattr(l.config,key) for key in l.config.__slots__}
  #l.config_log.append(config)
  #SaveYAML(l.config_log, l.logdir+'config_log.yaml', interactive=False)

def LoadActions(database, i_episode=0, i_node=0):
  xs= LoadYAML(database)['Entry'][i_episode]['Seq'][i_node]['XS']
  act_keys= (
    'gh_ratio','p_pour_trg0','p_pour_trg',
    'dtheta1','dtheta2','shake_spd','shake_axis2','skill',)
  actions= [ToList(np.array(xs[key]['X']).ravel()) for key in act_keys]
  return {key:(value[0] if len(value)==1 else value) for key,value in zip(act_keys,actions)}

def Run(ct,*args):
  l= TContainer(debug=True)
  l.opt_conf= {}
  l.opt_conf['config']= LoadYAML('/tmp/dpl03/config_log.yaml')[0]
  l.opt_conf['actions']= LoadActions('/tmp/dpl03/database.yaml')
  l.config_log= []
  l.config_callback= TestConfigCallback
  ct.Run('tsim2.setup', l)
  sim= ct.sim
  l= ct.sim_local

  obs_keys0= ('ps_rcv','p_pour','p_pour_z','lp_pour','a_trg','size_srcmouth','material2')
  obs_keys_after_grab= obs_keys0+('gh_abs',)
  obs_keys_before_flow= obs_keys_after_grab+('a_pour','a_spill2','a_total')
  obs_keys_after_flow= obs_keys_before_flow+('lp_flow2','lpp_flow','flow_var','da_pour','da_spill2','da_total')
  XS= []

  try:
    XS.append(ObserveXSSA(l,None,obs_keys0+('da_trg',)))
    ct.Run('tsim2.act.grab', l.opt_conf['actions'])
    XS.append(ObserveXSSA(l,XS[-1],obs_keys_after_grab))
    ct.Run('tsim2.act.move_to_rcv', l.opt_conf['actions'])
    XS.append(ObserveXSSA(l,XS[-1],obs_keys_after_grab+('dps_rcv','v_rcv','da_trg')))
    ct.Run('tsim2.act.move_to_pour', l.opt_conf['actions'])
    XS.append(ObserveXSSA(l,XS[-1],obs_keys_before_flow))

    if l.opt_conf['actions']['skill']==0:
      ct.Run('tsim2.act.std_pour', l.opt_conf['actions'])
    else:
      ct.Run('tsim2.act.shake_A', l.opt_conf['actions'])
    XS.append(ObserveXSSA(l,XS[-1],obs_keys_after_flow))

    SaveYAML(XS, '/tmp/tsim2log/%s.dat'%TimeStr('short2'))

  finally:
    sim.StopPubSub(ct,l)
    l.sensor_callback= None
    ct.srvp.ode_pause()

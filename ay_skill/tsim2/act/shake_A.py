#!/usr/bin/python
from core_tool import *
SmartImportReload('sm_tools')
from sm_tools import RunSMAsThread
def Help():
  return '''Primitive action (shake_A) for ODE grasping and pouring simulation.
  Usage: Expected to be used from an execution context.'''

def GenSMShakeA(ct,l,sim):
  sm= TStateMachine(debug=True, local_obj=l)
  #sm.EventCallback= ct.SMCallback

  sm.l.max_theta= 0.9*math.pi

  def Shake(count):
    spd= sm.l.shake_spd
    p_pour_init= Vec(copy.deepcopy(sm.l.sensors.p_pour))
    p_pour_2= p_pour_init + Vec(sm.l.shake_axis)
    for n in range(count):
      for i in range(100):
        if sim.MoveToTrgPPour(ct,sm.l,p_pour_2,spd=spd):
          break
      for i in range(100):
        if sim.MoveToTrgPPour(ct,sm.l,p_pour_init,spd=spd):
          break
      sim.SimSleep(ct,l,0.1)
  sm.l.Shake= Shake

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= EXIT_STATE

  spilled_action= TFSMConditionedAction()
  spilled_action.Condition= sm.l.IsSpilledEmpty
  spilled_action.NextState= EXIT_STATE

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= EXIT_STATE

  sm.StartState= 'to_max'

  #sm.NewState('to_initial')
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1]= poured_action
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1]= spilled_action
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1]= timeout_action
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1].Condition= lambda: sm.l.sensors.theta<=sm.l.theta_init
  #sm['to_initial'].Actions[-1].NextState= 'to_max'
  #sm['to_initial'].ElseAction.Condition= lambda: True
  #sm['to_initial'].ElseAction.Action= lambda: sim.MoveDTheta(ct,sm.l,-sm.l.dtheta_max)
  #sm['to_initial'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('to_max')
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1]= poured_action
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1]= spilled_action
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1]= timeout_action
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1].Condition= lambda: sm.l.sensors.theta>sm.l.max_theta
  sm['to_max'].Actions[-1].NextState= 'shake'
  sm['to_max'].ElseAction.Condition= lambda: True
  sm['to_max'].ElseAction.Action= lambda: sim.MoveDTheta(ct,sm.l,sm.l.dtheta1)
  sm['to_max'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('shake')
  sm['shake'].EntryAction= lambda: sm.l.ChargeTimer(3.0)
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= poured_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= spilled_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= timeout_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= lambda: sm.l.sensors.num_flow>0
  sm['shake'].Actions[-1].Action= lambda: ( sm.l.ChargeTimer(3.0), sm.l.Shake(2) )
  sm['shake'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= lambda: not (sm.l.sensors.time>sm.l.timer_tstop)
  sm['shake'].Actions[-1].Action= lambda: ( sm.l.Shake(2) )
  sm['shake'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake'].ElseAction.Condition= lambda: True
  sm['shake'].ElseAction.NextState= EXIT_STATE

  return sm


def GenSMPourHeightCtrl(ct,l,sim):
  sm= TStateMachine(debug=False, local_obj=l)
  #sm.EventCallback= ct.SMCallback  #WARNING: No callback to avoide confuse the context

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= 'stop'

  spilled_action= TFSMConditionedAction()
  spilled_action.Condition= sm.l.IsSpilledEmpty
  spilled_action.NextState= 'stop'

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'

  def Init():
    sm.l.p_pour_init= copy.deepcopy(sm.l.sensors.p_pour)

  def HeadCtrlStep():
    if sm.l.sensors.src_colliding or sm.l.sensors.gripper_colliding:
      #If collision
      sim.MoveToTrgPPour(ct,sm.l,sm.l.p_pour_trg0,spd=0.06)
    else:
      #sim.MoveToTrgPPour(ct,sm.l,sm.l.p_pour_trg,spd=0.06)
      sim.MoveToTrgPPour(ct,sm.l,sm.l.p_pour_init,spd=0.06)

  def StopStep():
    sim.MoveToTrgPPour(ct,sm.l,sm.l.p_pour_init,spd=0.1)
    #if sm.l.sensors.src_colliding or sm.l.sensors.gripper_colliding:
      ##If collision
      #sim.MoveToTrgPPour(ct,sm.l,sm.l.p_pour_trg0,spd=0.1)

  sm.NewState('init')
  sm['init'].EntryAction= Init
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'move'

  sm.NewState('move')
  sm['move'].NewAction()
  sm['move'].Actions[-1].Condition= lambda: not sm.ThreadInfo.IsRunning()
  sm['move'].Actions[-1].NextState= 'stop'
  sm['move'].NewAction()
  sm['move'].Actions[-1]= poured_action
  sm['move'].NewAction()
  sm['move'].Actions[-1]= spilled_action
  sm['move'].NewAction()
  sm['move'].Actions[-1]= timeout_action
  sm['move'].ElseAction.Condition= lambda: True
  sm['move'].ElseAction.Action= HeadCtrlStep
  sm['move'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('stop')
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: sm.ThreadInfo.IsRunning()
  sm['stop'].Actions[-1].Action= StopStep
  sm['stop'].Actions[-1].NextState= ORIGIN_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  return sm


def FlowCGenSM(ct,l,sim):
  sm= TStateMachine(debug=True)
  #sm.EventCallback= ct.SMCallback

  l.dtheta_max= 0.02

  l.ChargeTimer= lambda dt: setattr(l,'timer_tstop',l.sensors.time+dt)

  l.sub_sm= TContainer()
  l.sub_sm.shake_A= GenSMShakeA(ct,l,sim)
  l.sub_sm.height_ctrl= GenSMPourHeightCtrl(ct,l,sim)


  #timeout_action= TFSMConditionedAction()
  #timeout_action.Condition= l.IsTimeout
  #timeout_action.NextState= 'kickback'

  sm.StartState= 'init'
  sm.NewState('init')
  sm['init'].EntryAction= lambda: RunSMAsThread(ct,l.sub_sm.height_ctrl,'flowc_height_ctrl')
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'shake_A'

  sm.NewState('shake_A')
  sm['shake_A'].EntryAction= lambda: l.sub_sm.shake_A.Run()
  sm['shake_A'].ElseAction.Condition= lambda: True
  #sm['shake_A'].ElseAction.Action= lambda: l.planlearn_callback(ct,l,sim,'shake_A_end')
  sm['shake_A'].ElseAction.NextState= 'kickback'

  sm.NewState('kickback')
  sm['kickback'].NewAction()
  sm['kickback'].Actions[-1].Condition= lambda: l.sensors.theta<=l.theta_init
  sm['kickback'].Actions[-1].Action= lambda: ( l.sub_sm.height_ctrl.ThreadInfo.Stop(),
                                              Print('End of pouring') )
  sm['kickback'].Actions[-1].NextState= 'shift_to_pour_l0'
  sm['kickback'].ElseAction.Condition= lambda: True
  sm['kickback'].ElseAction.Action= lambda: sim.MoveDTheta(ct,l,-l.dtheta_max)
  sm['kickback'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('shift_to_pour_l0')
  sm['shift_to_pour_l0'].EntryAction= lambda: setattr(l,'shift_to_pour_l0_reached',False)
  sm['shift_to_pour_l0'].NewAction()
  sm['shift_to_pour_l0'].Actions[-1].Condition= lambda: not l.shift_to_pour_l0_reached
  sm['shift_to_pour_l0'].Actions[-1].Action= lambda: setattr(l,'shift_to_pour_l0_reached',
                                                             sim.MoveToTrgPPour(ct,l,l.p_pour_trg0,spd=0.1))
  sm['shift_to_pour_l0'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_pour_l0'].ElseAction.Condition= lambda: True
  sm['shift_to_pour_l0'].ElseAction.NextState= EXIT_STATE

  sm.Run()

  for sub_sm in l.sub_sm.values():
    sub_sm.Cleanup()
  sm.Cleanup()

  return sm.ExitStatus

def Run(ct,*args):
  params= args[0]  #Parameters (dictionary) of this action

  sim= ct.sim
  l= ct.sim_local

  #l.dtheta1= l.xs.n2c['dtheta1'].X[0,0]
  #l.shake_spd= l.xs.n2c['shake_spd'].X[0,0]
  #shake_axis= ToList(l.xs.n2c['shake_axis2'].X)
  l.dtheta1= params['dtheta1']
  l.shake_spd= params['shake_spd']
  shake_axis= params['shake_axis2']
  l.shake_axis= [shake_axis[0]*math.sin(shake_axis[1]), 0.0, shake_axis[0]*math.cos(shake_axis[1])]

  l.flow_controlling= True
  l.exec_status= FlowCGenSM(ct,l,sim)
  #l.planlearn_callback(ct,l,sim,'end_of_flowc_gen')
  l.flow_controlling= False
  #Log('After end_of_flowc_gen')

  return (SUCCESS_CODE, FAILURE_PRECOND, FAILURE_OTHER)[0]

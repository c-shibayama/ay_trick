#!/usr/bin/python
from core_tool import *
SmartImportReload('lib.sm_tools')
from lib.sm_tools import RunSMAsThread
def Help():
  return '''Primitive action (std_pour) for ODE grasping and pouring simulation.
  Usage: Expected to be used from an execution context.'''

def GenSMStdPour(ct,l,sim):
  sm= TStateMachine(debug=True, local_obj=l)
  #sm.EventCallback= ct.SMCallback

  sm.l.theta_flowstart= 0.5*math.pi
  sm.l.max_theta= 0.9*math.pi

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= EXIT_STATE

  spilled_action= TFSMConditionedAction()
  spilled_action.Condition= sm.l.IsSpilledEmpty
  spilled_action.NextState= EXIT_STATE

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= EXIT_STATE

  sm.StartState= 'approach'

  #sm.NewState('to_initial')
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1]= poured_action
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1]= spilled_action
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1]= timeout_action
  #sm['to_initial'].NewAction()
  #sm['to_initial'].Actions[-1].Condition= lambda: sm.l.sensors.theta<=sm.l.theta_init
  #sm['to_initial'].Actions[-1].NextState= 'approach'  #NEW_TFlowAmountModel
  #sm['to_initial'].ElseAction.Condition= lambda: True
  #sm['to_initial'].ElseAction.Action= lambda: sim.MoveDTheta(ct,sm.l,-sm.l.dtheta_max)
  #sm['to_initial'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('approach')  #NEW_TFlowAmountModel
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= poured_action
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= spilled_action
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= timeout_action
  sm['approach'].NewAction()
  sm['approach'].NewAction()
  sm['approach'].Actions[-1].Condition= lambda: sm.l.sensors.theta>sm.l.theta_flowstart
  sm['approach'].Actions[-1].NextState= 'find_flow_p'
  sm['approach'].ElseAction.Condition= lambda: True
  sm['approach'].ElseAction.Action= lambda: sim.MoveDTheta(ct,sm.l,sm.l.dtheta1)
  sm['approach'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('find_flow_p')
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= poured_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= spilled_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= timeout_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.sensors.theta>sm.l.max_theta
  sm['find_flow_p'].Actions[-1].NextState= EXIT_STATE
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.sensors.num_flow>0
  sm['find_flow_p'].Actions[-1].NextState= 'pour'
  sm['find_flow_p'].ElseAction.Condition= lambda: True
  sm['find_flow_p'].ElseAction.Action= lambda: sim.MoveDTheta(ct,sm.l,sm.l.dtheta2)
  sm['find_flow_p'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('pour')
  sm['pour'].EntryAction= lambda: sm.l.ChargeTimer(5.0)  #Time of patience
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= spilled_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: sm.l.sensors.theta>sm.l.max_theta
  sm['pour'].Actions[-1].NextState= EXIT_STATE
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: sm.l.sensors.num_flow>0
  sm['pour'].Actions[-1].Action= lambda: ( sm.l.ChargeTimer(5.0), sim.MoveDTheta(ct,sm.l,0.0) )
  sm['pour'].Actions[-1].NextState= ORIGIN_STATE
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: sm.l.sensors.time>sm.l.timer_tstop
  sm['pour'].Actions[-1].NextState= EXIT_STATE
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: sim.MoveDTheta(ct,sm.l,sm.l.dtheta2)
  sm['pour'].ElseAction.NextState= ORIGIN_STATE

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
  l.sub_sm.std_pour= GenSMStdPour(ct,l,sim)
  l.sub_sm.height_ctrl= GenSMPourHeightCtrl(ct,l,sim)


  #timeout_action= TFSMConditionedAction()
  #timeout_action.Condition= l.IsTimeout
  #timeout_action.NextState= 'kickback'

  sm.StartState= 'init'
  sm.NewState('init')
  sm['init'].EntryAction= lambda: RunSMAsThread(ct,l.sub_sm.height_ctrl,'flowc_height_ctrl')
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'std_pour'

  sm.NewState('std_pour')
  sm['std_pour'].EntryAction= lambda: l.sub_sm.std_pour.Run()
  sm['std_pour'].ElseAction.Condition= lambda: True
  #sm['std_pour'].ElseAction.Action= lambda: l.planlearn_callback(ct,l,sim,'std_pour_end')
  sm['std_pour'].ElseAction.NextState= 'kickback'

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
  #l.dtheta2= l.xs.n2c['dtheta2'].X[0,0]
  l.dtheta1= params['dtheta1']
  l.dtheta2= params['dtheta2']

  l.flow_controlling= True
  l.exec_status= FlowCGenSM(ct,l,sim)
  #l.planlearn_callback(ct,l,sim,'end_of_flowc_gen')
  l.flow_controlling= False
  #Log('After end_of_flowc_gen')

  return (SUCCESS_CODE, FAILURE_PRECOND, FAILURE_OTHER)[0]

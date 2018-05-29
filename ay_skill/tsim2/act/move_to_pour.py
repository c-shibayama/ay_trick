#!/usr/bin/python
from core_tool import *
SmartImportReload('tsim.sm4')
from tsim.sm4 import (
  ResetFilter, ApplyFilter,
  )
def Help():
  return '''Primitive action (move_to_pour) for ODE grasping and pouring simulation.
  Usage: Expected to be used from an execution context.'''

#Move to pouring point state machine
def MoveToPourSM(ct,l,sim):
  sm= TStateMachine()
  #sm.EventCallback= ct.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'shift_to_pour_l'

  sm.NewState('shift_to_pour_l')
  sm['shift_to_pour_l'].EntryAction= lambda: setattr(l,'shift_to_pour_l_reached',False)
  sm['shift_to_pour_l'].NewAction()
  sm['shift_to_pour_l'].Actions[-1].Condition= lambda: not l.shift_to_pour_l_reached and not l.sensors.src_colliding and not l.sensors.gripper_colliding
  sm['shift_to_pour_l'].Actions[-1].Action= lambda: setattr(l,'shift_to_pour_l_reached',
                                                            sim.MoveToTrgPPour(ct,l,l.p_pour_trg,spd=0.1))
  sm['shift_to_pour_l'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_pour_l'].ElseAction.Condition= lambda: True
  sm['shift_to_pour_l'].ElseAction.NextState= EXIT_STATE

  sm.Run()
  l= None
  return sm.ExitStatus

def Run(ct,*args):
  params= args[0]  #Parameters (dictionary) of this action

  sim= ct.sim
  l= ct.sim_local

  ResetFilter(l)  #TODO:FIXME: This action should be reasoned
  ApplyFilter(ct,l,sim)

  #Plan pour
  #l.planlearn_callback(ct,l,sim,'infer_move_to_pour')
  #Log('After infer_move_to_pour')

  #pp= ToList(l.xs.n2a['p_pour_trg'].X)
  pp= params['p_pour_trg']
  l.p_pour_trg= [pp[0], 0.0, pp[1]]
  l.exec_status= SUCCESS_CODE
  #l.user_viz.pop()
  #VizPP(l,l.p_pour_trg,[1.,0.,1.])

  l.exec_status= MoveToPourSM(ct,l,sim)
  sim.SimSleep(ct,l,0.2)  #Wait for container movement when colliding
  #l.planlearn_callback(ct,l,sim,'end_of_move_to_pour')
  #Log('After end_of_move_to_pour')

  return l.exec_status

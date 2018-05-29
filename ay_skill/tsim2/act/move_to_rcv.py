#!/usr/bin/python
from core_tool import *
import std_msgs.msg
def Help():
  return '''Primitive action (move_to_rcv) for ODE grasping and pouring simulation.
  Usage: Expected to be used from an execution context.'''

#Move to around receiver position state machine
def MoveToRcvSM(ct,l,sim):
  def MoveToPourL0():
    p_pour0= copy.deepcopy(l.sensors.p_pour)
    theta0= l.sensors.theta
    p_pour_msg= std_msgs.msg.Float64MultiArray()
    theta_msg= std_msgs.msg.Float64()
    Ndiv= max(5, int(Dist(l.p_pour_trg0,p_pour0)*50))
    for tc in FRange1(0.0,1.0,Ndiv):
      p_pour_msg.data= (1.0-tc)*Vec(p_pour0) + tc*Vec(l.p_pour_trg0)
      theta_msg.data= (1.0-tc)*theta0 + tc*l.theta_init
      ct.pub.ode_ppour.publish(p_pour_msg)
      ct.pub.ode_theta.publish(theta_msg)
      sim.SimSleep(ct,l,0.04)
      #if l.sensors.src_colliding or l.sensors.gripper_colliding:
        #break
    l.exec_status= SUCCESS_CODE

  sm= TStateMachine()
  #sm.EventCallback= ct.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'move_upward'

  sm.NewState('move_upward')
  sm['move_upward'].NewAction()
  sm['move_upward'].Actions[-1].Condition= lambda: l.sensors.src_colliding
  sm['move_upward'].Actions[-1].Action= lambda: sim.MoveDPPour(ct,l,[0.0,0.0,0.01])
  sm['move_upward'].Actions[-1].NextState= ORIGIN_STATE
  sm['move_upward'].ElseAction.Condition= lambda: True
  sm['move_upward'].ElseAction.NextState= 'move_to_pour_l0'

  sm.NewState('move_to_pour_l0')
  sm['move_to_pour_l0'].EntryAction= MoveToPourL0
  sm['move_to_pour_l0'].NewAction()
  sm['move_to_pour_l0'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_pour_l0'].Actions[-1].NextState= EXIT_STATE
  sm['move_to_pour_l0'].ElseAction.Condition= lambda: True
  sm['move_to_pour_l0'].ElseAction.Action= sm.SetFailure
  sm['move_to_pour_l0'].ElseAction.NextState= EXIT_STATE

  sm.Run()
  l= None
  return sm.ExitStatus

def Run(ct,*args):
  params= args[0]  #Parameters (dictionary) of this action

  sim= ct.sim
  l= ct.sim_local

  #Plan pour
  #l.planlearn_callback(ct,l,sim,'infer_move_to_rcv')
  #Log('After infer_move_to_rcv')

  #pp0= ToList(l.xs.n1['p_pour_trg0'].X)
  #pp= ToList(l.xs.n1['p_pour_trg'].X)
  pp0= params['p_pour_trg0']
  #pp= params['p_pour_trg']
  l.p_pour_trg0= [pp0[0], 0.0, pp0[1]]
  l.theta_init= DegToRad(45.0)  #<-- should be planned
  l.exec_status= SUCCESS_CODE
  #VizPP(l,l.p_pour_trg0,[0.,1.,0.])
  #VizPP(l,[pp[0], 0.0, pp[1]],[0.5,0.,1.])

  l.exec_status= MoveToRcvSM(ct,l,sim)
  sim.SimSleep(ct,l,0.2)  #Wait for container movement when colliding
  #l.planlearn_callback(ct,l,sim,'end_of_move_to_rcv')
  #Log('After end_of_move_to_rcv')

  return l.exec_status

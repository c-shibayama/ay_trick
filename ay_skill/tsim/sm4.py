#!/usr/bin/python
from core_tool import *
import std_msgs.msg
import std_srvs.srv
roslib.load_manifest('ay_sim_msgs')
import ay_sim_msgs.msg
SmartImportReload('sm_tools')
from sm_tools import RunSMAsThread

def Help():
  return '''State machine for ODE grasping and pouring simulation (ver.4).
    ver.2: Material type is introduced.  Shaking is introduced.
    ver.3: In try-and-error of skill selection, pouring location is re-planned.
    ver.4: Skill parameters are modified to be planned.
  Usage: tsim.sm4'''

#preset: 'bounce','nobounce','natto','ketchup','slime',None
#kind: 'smooth','viscous',None
def SetMaterial(l, preset=None, kind=None, bounce1=0.1, bounce2=0.2, viscous1=2.5e-7, viscous2=0.2):
  l.config.ContactBounce= 0.1
  l.config.ContactBounceVel= 0.2
  l.config.ViscosityParam1= 0.0  #Default: 0.0
  l.config.ViscosityMaxDist= 0.1  #Default: 0.1
  if preset is not None:
    if preset=='bounce':
      #Bounce balls:
      l.config.ContactBounce= 0.7
      l.config.ContactBounceVel= 0.2
      l.config.ViscosityParam1= 0.0  #Default: 0.0
    elif preset=='nobounce':
      #Non-bounce balls:
      l.config.ContactBounce= 0.1
      l.config.ContactBounceVel= 0.2
      l.config.ViscosityParam1= 0.0  #Default: 0.0
    elif preset=='natto':
      #Natto:
      l.config.ContactBounce= 0.1
      l.config.ContactBounceVel= 0.01
      l.config.ViscosityParam1= 1.5e-6  #Default: 0.0
      l.config.ViscosityMaxDist= 0.1  #Default: 0.1
    elif preset=='ketchup':
      #Ketchup:
      l.config.ContactBounce= 0.1
      l.config.ContactBounceVel= 0.01
      l.config.ViscosityParam1= 2.5e-7  #Default: 0.0
      l.config.ViscosityMaxDist= 0.2  #Default: 0.1
    elif preset=='slime':
      #Slime???:
      l.config.ContactBounce= 0.1
      l.config.ContactBounceVel= 0.01
      l.config.ViscosityParam1= 1.0e-5  #Default: 0.0
      l.config.ViscosityMaxDist= 0.1  #Default: 0.1
    else:  raise Exception('SetMaterial: unknown preset:',preset)
  elif kind is not None:
    if kind=='smooth':
      #Smooth material
      l.config.ContactBounce= bounce1
      l.config.ContactBounceVel= bounce2
      l.config.ViscosityParam1= 0.0  #Default: 0.0
    elif kind=='viscous':
      l.config.ContactBounce= 0.1
      l.config.ContactBounceVel= 0.01
      l.config.ViscosityParam1= viscous1  #Default: 0.0
      l.config.ViscosityMaxDist= viscous2  #Default: 0.1
    else:  raise Exception('SetMaterial: unknown kind:',kind)
  else:
    l.config.ContactBounce= bounce1
    l.config.ContactBounceVel= bounce2
    l.config.ViscosityParam1= viscous1  #Default: 0.0
    l.config.ViscosityMaxDist= viscous2  #Default: 0.1

def ResetFilter(l):
  l.filtered= TContainer(debug=True)

def ApplyFilter(ct,l,sim):
  if 'num_spill' not in l.filtered:  l.filtered.num_spill= l.sensors.num_spill
  else:  l.filtered.num_spill= max(l.filtered.num_spill, l.sensors.num_spill)
  if 'dnum_spill' not in l.filtered:
    l.filtered.num_spill0= l.filtered.num_spill
    l.filtered.dnum_spill= 0
  else:
    l.filtered.dnum_spill= l.filtered.num_spill - l.filtered.num_spill0
  if 'num_bounce' not in l.filtered:  l.filtered.num_bounce= l.sensors.num_bounce
  else:  l.filtered.num_bounce= max(l.filtered.num_bounce, l.sensors.num_bounce)
  #l.filtered.amount= l.sensors.z_rcv*5.0 if l.sensors.z_rcv>0.0 else 0.0
  l.filtered.amount= 0.0055*l.sensors.num_rcv

  if 'v_rcv' not in l.filtered:
    l.filtered.v_rcv= [0.0, 0.0, 0.0]
    l.filtered.p_rcv_prev= [l.sensors.x_rcv.position.x, l.sensors.x_rcv.position.y, l.sensors.x_rcv.position.z]
  p_rcv= [l.sensors.x_rcv.position.x, l.sensors.x_rcv.position.y, l.sensors.x_rcv.position.z]
  l.filtered.v_rcv= [(p_rcv[d]-l.filtered.p_rcv_prev[d])/l.config.TimeStep for d in range(3)]

  ## Ball status kinds (1:in src, 2:in rcv, 3:flow, 4:spill, 0:unknown)
  #if 'ball_st' not in l.filtered:
    ##If only st==1 (in src) is in l.sensors.ball_st:
    #if [st for st in l.sensors.ball_st if st!=1]==[]:
      #l.filtered.ball_st= [1]*len(l.sensors.ball_st)
  #else:

  if l.flow_controlling:
    if 'term_flow_x' not in l.filtered:
      l.filtered.term_flow_x= []
      l.filtered.term_flow_center= [0.0,0.0,0.0]
      l.filtered.term_flow_var= 0.0
      l.filtered.term_flow_max_dist= 0.0
      l.filtered.obs_counter= 0
    #l.sensors.ball_st, l.sensors.ball_x
    flow_x= [l.sensors.ball_x[6*i:6*(i+1)] for i in range(len(l.sensors.ball_st)) if l.sensors.ball_st[i]==3]
    #Note: ball_st[i]==3 denotes a flow particle
    #flow_x.sort(key=lambda x:x[2])  #sort by z
    #N= 10
    #term_flow_x= flow_x[:N]  #particles on the floor or the bottom of the receiver
    l.filtered.term_flow_x+= [xf for xf in flow_x if xf[2]<0.1]  #particles whose height is < 10 cm
    random.shuffle(l.filtered.term_flow_x)
    N= 50
    l.filtered.term_flow_x= l.filtered.term_flow_x[:N]
    term_flow_x= l.filtered.term_flow_x
    N= len(term_flow_x)
    if N>0:
      #term_flow_x.sort(key=lambda x:x[2])  #sort by z
      term_flow_center= [sum([term_flow_x[i][0] for i in range(N)])/float(N),
                        sum([term_flow_x[i][1] for i in range(N)])/float(N),
                        #sum([term_flow_x[i][2] for i in range(N)])/float(N)
                        0.0 ]
      dist_x_data= [(Dist(term_flow_x[i][:3], term_flow_center), term_flow_x[i]) for i in range(N)]
      dist_x_data.sort(reverse=True, key=lambda x:x[0])
      term_flow_var= sum([d*d for d,x in dist_x_data]) / float(N)
      term_flow_max_dist= dist_x_data[0][0]
      #term_flow_center : center of flow
      #dist_x_data[0][0] : most far flow
      #dist_x_data[0][1] : corresponding point
      l.filtered.obs_counter+= 1
      if l.filtered.term_flow_center==[0.0,0.0,0.0]:
        l.filtered.term_flow_center= term_flow_center
      else:
        alpha= 0.5/(0.1*l.filtered.obs_counter+1.0)
        l.filtered.term_flow_center= [(1.0-alpha)*f0+alpha*f1 for f0,f1 in zip(l.filtered.term_flow_center,term_flow_center)]
      l.filtered.term_flow_var= min(max(l.filtered.term_flow_var, term_flow_var), 1.0)
      l.filtered.term_flow_max_dist= min(max(l.filtered.term_flow_max_dist, term_flow_max_dist), 1.0)

  #Visualize flow:
  if 'term_flow_max_dist' in l.filtered and l.filtered.term_flow_max_dist>0.0:
    msg= ay_sim_msgs.msg.ODEViz()
    if 'user_viz' in l:
      msg.objects+= l.user_viz
    prm= ay_sim_msgs.msg.ODEVizPrimitive()
    prm.type= prm.LINE
    prm.pose.position.x= l.filtered.term_flow_center[0]
    prm.pose.position.y= l.filtered.term_flow_center[1]
    prm.pose.position.z= l.filtered.term_flow_center[2]
    prm.param= [0.0,0.0,0.3]
    prm.color.r= 0.0
    prm.color.g= 1.0
    prm.color.b= 1.0
    prm.color.a= 0.2
    msg.objects.append(prm)
    prm= ay_sim_msgs.msg.ODEVizPrimitive()
    prm.type= prm.CYLINDER
    prm.pose.position.x= l.filtered.term_flow_center[0]
    prm.pose.position.y= l.filtered.term_flow_center[1]
    prm.pose.position.z= l.filtered.term_flow_center[2]
    prm.param= [math.sqrt(l.filtered.term_flow_var), 0.015]
    prm.color.r= 1.0
    prm.color.g= 1.0
    prm.color.b= 0.0
    prm.color.a= 0.1
    msg.objects.append(prm)
    prm= ay_sim_msgs.msg.ODEVizPrimitive()
    prm.type= prm.CYLINDER
    prm.pose.position.x= l.filtered.term_flow_center[0]
    prm.pose.position.y= l.filtered.term_flow_center[1]
    prm.pose.position.z= l.filtered.term_flow_center[2]
    prm.param= [l.filtered.term_flow_max_dist, 0.007]
    prm.color.r= 1.0
    prm.color.g= 1.0
    prm.color.b= 0.0
    prm.color.a= 0.1
    msg.objects.append(prm)
    ct.pub.ode_viz.publish(msg)
  elif 'user_viz' in l:
    msg= ay_sim_msgs.msg.ODEViz()
    msg.objects+= l.user_viz
    ct.pub.ode_viz.publish(msg)

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
  l.sub_sm.std_pour= GenSMStdPour(ct,l,sim)
  l.sub_sm.shake_A= GenSMShakeA(ct,l,sim)
  l.sub_sm.height_ctrl= GenSMPourHeightCtrl(ct,l,sim)


  #timeout_action= TFSMConditionedAction()
  #timeout_action.Condition= l.IsTimeout
  #timeout_action.NextState= 'kickback'

  sm.StartState= 'init'
  sm.NewState('init')
  sm['init'].EntryAction= lambda: RunSMAsThread(ct,l.sub_sm.height_ctrl,'flowc_height_ctrl')
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'start'

  sm.NewState('start')
  sm['start'].EntryAction= lambda: l.planlearn_callback(ct,l,sim,'select_skill')
  #sm['start'].NewAction()
  #sm['start'].Actions[-1].Condition= l.IsPoured
  #sm['start'].Actions[-1].NextState= 'kickback'
  #sm['start'].NewAction()
  #sm['start'].Actions[-1].Condition= l.IsSpilledEmpty
  #sm['start'].Actions[-1].NextState= 'kickback'
  #sm['start'].NewAction()
  #sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: l.selected_skill=='std_pour'
  #sm['start'].Actions[-1].Action= lambda: SetBehavior('std_pour')
  sm['start'].Actions[-1].NextState= 'std_pour'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: l.selected_skill=='shake_A'
  #sm['start'].Actions[-1].Action= lambda: SetBehavior('shake_A')
  sm['start'].Actions[-1].NextState= 'shake_A'

  sm.NewState('std_pour')
  sm['std_pour'].EntryAction= lambda: l.sub_sm.std_pour.Run()
  sm['std_pour'].ElseAction.Condition= lambda: True
  sm['std_pour'].ElseAction.Action= lambda: l.planlearn_callback(ct,l,sim,'std_pour_end')
  sm['std_pour'].ElseAction.NextState= 'kickback'

  sm.NewState('shake_A')
  sm['shake_A'].EntryAction= lambda: l.sub_sm.shake_A.Run()
  sm['shake_A'].ElseAction.Condition= lambda: True
  sm['shake_A'].ElseAction.Action= lambda: l.planlearn_callback(ct,l,sim,'shake_A_end')
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


def PourSM(ct,l):
  sim= ct.Load('tsim.core1')

  def Log(msg):
    if 'sm_logfp' in l and l.sm_logfp is not None:  out= l.sm_logfp
    else:  out= sys.stdout
    out.write('-------------------\n')
    out.write('======== LOGENTRY: %s ========\n'%msg)
    out.write('<<<l.config>>>\n')
    out.write( str(l.config) )
    out.write('\n<<<l>>>\n')
    for key in ('p_pour_trg','p_pour_trg0','theta_init'):
      out.write('l.{key}: {x}\n'.format(key=key,x=l[key] if key in l else 'N/A'))
    out.write('-------------------\n')
    out.flush()

  def Setup():
    sim.SetupServiceProxy(ct,l)
    sim.SetupPubSub(ct,l)

    l.max_duration= 500.0
    l.amount_trg= 0.3
    l.spilled_stop= 5

    l.IsTimeout= lambda: (l.sensors.time-l.start_time > l.max_duration)
    l.IsPoured= lambda: (l.filtered.amount > l.amount_trg)
    l.IsEmpty= lambda: (l.sensors.num_src <= 20)
    l.IsSpilled= lambda: (l.filtered.dnum_spill >= l.spilled_stop)
    #l.IsSpilled= lambda: (Print('dnum_spill=',l.filtered.dnum_spill,l.spilled_stop,(l.filtered.dnum_spill >= l.spilled_stop)),
                          #(l.filtered.dnum_spill >= l.spilled_stop))[-1]
    l.IsSpilledEmpty= lambda: l.IsSpilled() or l.IsEmpty()

    ct.srvp.ode_resume()
    l.config= sim.GetConfig(ct)
    print 'Current config:',l.config

    #Setup config
    l.config.MaxContacts= 2
    l.config.TimeStep= 0.025
    l.config.Gravity= -1.0
    l.config.BallType= 0  #Sphere particles
    #l.config.BallType= 1  #Box particles
    l.config.SrcSize2H= 0.08  #Mouth size; Default: 0.03
    SetMaterial(l, preset='ketchup')
    if 'config_callback' in l and l.config_callback!=None:
      l.config_callback(ct,l,sim)
    Log('After l.config_callback')
    l.config.ViscosityParam1= 0.0

    #Reset to get state for plan
    sim.ResetConfig(ct,l.config)
    time.sleep(0.1)  #Wait for l.sensors is updated
    #ct.srvp.ode_pause()  #Pause to wait grasp plan

    ResetFilter(l)
    l.flow_controlling= False
    l.sensor_callback= lambda:ApplyFilter(ct,l,sim)  #Activate filter

  def Grab():
    #Plan grasp
    l.planlearn_callback(ct,l,sim,'infer_grab')
    Log('After infer_grab')

    #ct.srvp.ode_resume()
    #Reset again to apply the grasp plan
    sim.ResetConfig(ct,l.config)
    Log('After sim.ResetConfig in Grab')
    sim.SimSleep(ct,l,0.1)  #Wait for reset action is done
    l.exec_status= SUCCESS_CODE

  def MoveToRcv():
    #Plan pour
    l.planlearn_callback(ct,l,sim,'infer_move_to_rcv')
    Log('After infer_move_to_rcv')

    l.exec_status= MoveToRcvSM(ct,l,sim)
    sim.SimSleep(ct,l,0.2)  #Wait for container movement when colliding
    l.planlearn_callback(ct,l,sim,'end_of_move_to_rcv')
    Log('After end_of_move_to_rcv')

    l.start_time= l.sensors.time

  def MoveToPour():
    ResetFilter(l)
    ApplyFilter(ct,l,sim)

    #Plan pour
    l.planlearn_callback(ct,l,sim,'infer_move_to_pour')
    Log('After infer_move_to_pour')

    l.exec_status= MoveToPourSM(ct,l,sim)
    sim.SimSleep(ct,l,0.2)  #Wait for container movement when colliding
    l.planlearn_callback(ct,l,sim,'end_of_move_to_pour')
    Log('After end_of_move_to_pour')

  def FlowCGen():
    l.flow_controlling= True
    l.exec_status= FlowCGenSM(ct,l,sim)
    l.planlearn_callback(ct,l,sim,'end_of_flowc_gen')
    l.flow_controlling= False
    Log('After end_of_flowc_gen')


  sm= TStateMachine()
  #sm.EventCallback= lambda sm,et,st,ac: ct.SMCallback(sm,et,st,ac)
  sm.Debug= True

  sm.StartState= 'start'

  sm.NewState('start')
  sm['start'].EntryAction= Setup
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'grab'

  sm.NewState('grab')
  sm['grab'].EntryAction= Grab
  sm['grab'].NewAction()
  sm['grab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['grab'].Actions[-1].NextState= 'move_to_rcv'
  sm['grab'].ElseAction.Condition= lambda: True
  sm['grab'].ElseAction.Action= sm.SetFailure
  sm['grab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('move_to_rcv')
  sm['move_to_rcv'].EntryAction= MoveToRcv
  sm['move_to_rcv'].NewAction()
  sm['move_to_rcv'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_rcv'].Actions[-1].NextState= 'move_to_pour'
  sm['move_to_rcv'].ElseAction.Condition= lambda: True
  sm['move_to_rcv'].ElseAction.Action= sm.SetFailure
  sm['move_to_rcv'].ElseAction.NextState= EXIT_STATE

  sm.NewState('move_to_pour')
  sm['move_to_pour'].EntryAction= MoveToPour
  sm['move_to_pour'].NewAction()
  sm['move_to_pour'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_pour'].Actions[-1].NextState= 'flowc_gen'
  sm['move_to_pour'].ElseAction.Condition= lambda: True
  sm['move_to_pour'].ElseAction.Action= sm.SetFailure
  sm['move_to_pour'].ElseAction.NextState= EXIT_STATE

  sm.NewState('flowc_gen')
  sm['flowc_gen'].EntryAction= FlowCGen
  sm['flowc_gen'].NewAction()
  sm['flowc_gen'].Actions[-1].Condition= lambda:l.IsTimeout() or l.IsEmpty()  # or l.IsSpilled()
  sm['flowc_gen'].Actions[-1].NextState= EXIT_STATE
  sm['flowc_gen'].NewAction()
  sm['flowc_gen'].Actions[-1].Condition= lambda:not IsSuccess(l.exec_status)
  sm['flowc_gen'].Actions[-1].Action= sm.SetFailure
  sm['flowc_gen'].Actions[-1].NextState= EXIT_STATE
  sm['flowc_gen'].NewAction()
  sm['flowc_gen'].Actions[-1].Condition= lambda:l.IsPoured()
  sm['flowc_gen'].Actions[-1].NextState= EXIT_STATE
  sm['flowc_gen'].ElseAction.Condition= lambda: True
  sm['flowc_gen'].ElseAction.NextState= 'move_to_pour'

  try:
    sm.Run()

  except Exception as e:
    PrintException(e, ' in tsim.sm4')

  finally:
    sim.StopPubSub(ct,l)
    l.sensor_callback= None
    ct.srvp.ode_pause()

  return sm.ExitStatus


def TestPlanLearnCallback(ct,l,sim,context):
  if context=='infer_grab':
    #Plan l.config.GripperHeight
    l.config.GripperHeight= 0.5*l.sensors.p_pour[2]  #Should be in [0,l.sensors.p_pour[2]]
  elif context=='infer_move_to_rcv':
    #Plan l.p_pour_trg0, l.theta_init
    l.p_pour_trg0= [l.sensors.x_rcv.position.x-0.1, 0.0, l.sensors.x_rcv.position.z+0.4+0.2]
    l.theta_init= DegToRad(45.0)
  elif context=='end_of_move_to_rcv':
    pass
  elif context=='infer_move_to_pour':
    #Plan l.p_pour_trg
    l.p_pour_trg= [l.sensors.x_rcv.position.x-0.1, 0.0, l.sensors.x_rcv.position.z+0.4]
  elif context=='end_of_move_to_pour':
    pass
  elif context=='select_skill':
    #Plan l.selected_skill from ('std_pour','shake_A','shake_B')
    #For 'std_pour' plan: l.dtheta1, l.dtheta2
    #For 'shake_A' plan: l.dtheta1, l.shake_spd, l.shake_axis
    l.selected_skill= ('std_pour','shake_A')[RandI(2)]
    if l.selected_skill=='std_pour':
      l.dtheta1= Rand(0.01, 0.03)   #Originally 0.7*sm.l.dtheta_max=0.014
      l.dtheta2= Rand(0.005, 0.03)  #Originally 0.2*sm.l.dtheta_max=0.004
    elif l.selected_skill=='shake_A':
      l.dtheta1= Rand(0.01, 0.03)   #Originally 0.7*sm.l.dtheta_max=0.014
      l.shake_spd= Rand(0.5, 1.0)   #Originally 0.8
      l.shake_axis= [Rand(0.0, 0.1), 0.0, Rand(0.0, 0.1)]  #Originally [0.0,0.0,0.08]
  elif context=='std_pour_end':
    pass
  elif context=='shake_A_end':
    pass
  elif context=='end_of_flowc_gen':
    pass
  l.exec_status= SUCCESS_CODE

def TestConfigCallback(ct,l,sim):
  #l.config.RcvPos= [0.6, l.config.RcvPos[1], l.config.RcvPos[2]]
  l.config.RcvPos= [0.8, l.config.RcvPos[1], l.config.RcvPos[2]]
  #l.config.RcvPos= [1.2, l.config.RcvPos[1], l.config.RcvPos[2]]
  #l.config.RcvPos= [0.8+0.6*(random.random()-0.5), l.config.RcvPos[1], l.config.RcvPos[2]]
  CPrint(3,'l.config.RcvPos=',l.config.RcvPos)
  #l.config.ContactBounce= 0.7
  #l.config.ContactBounce= 0.1+0.8*(random.random())
  #CPrint(3,'l.config.ContactBounce=',l.config.ContactBounce)


def Run(ct,*args):
  l= TContainer(debug=True)

  l.planlearn_callback= TestPlanLearnCallback
  l.config_callback= TestConfigCallback

  res= PourSM(ct,l)
  #print 'l.filtered=\n',l.filtered
  print 'term_flow_center=', l.filtered.term_flow_center
  print 'term_flow_var=', l.filtered.term_flow_var
  print 'term_flow_max_dist=', l.filtered.term_flow_max_dist
  l= None
  return res


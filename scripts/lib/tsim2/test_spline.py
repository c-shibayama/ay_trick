#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test spline in ODE grasping and pouring simulation.
  Usage: tsim2.test_spline'''


#Generate key points: [ct,x-1,x-2,...,x-Nd]*Ns
def GenKeyPoints(x0=[0.0],xf=[1.0],tf=1.0,param=[[0.0,0.0]]):
  Nd= len(x0)
  data= [[0.0]+[None]*Nd,
         [0.333*tf]+[None]*Nd,
         [0.666*tf]+[None]*Nd,
         [tf]+[None]*Nd]
  for d in range(Nd):
    a= (xf[d]-x0[d])/tf
    data[0][1+d]= x0[d]
    data[1][1+d]= x0[d]+a*data[1][0] + param[d][0]
    data[2][1+d]= x0[d]+a*data[2][0] + param[d][1]
    data[3][1+d]= xf[d]
  return data

def RunSpline(ct,l,sim,param):
  # Spline of velocity
  key_points= GenKeyPoints(x0=[0.0]*3, xf=[0.0]*3, tf=0.5,
                param=[param[0:2],param[2:4],param[4:6]])
  splines= [TCubicHermiteSpline() for d in range(len(key_points[0])-1)]
  for d in range(len(splines)):
    data_d= [[p[0],p[d+1]] for p in key_points]
    splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)
  dt= l.config.TimeStep
  t_curr= key_points[0][0]
  while True:
    q= [splines[d].Evaluate(t_curr) for d in range(len(splines))]
    dp= [q[0]*dt,0.0,q[1]*dt]
    dth= q[2]*dt
    if l.sensors.theta+dth>math.pi or l.sensors.theta+dth<0.0: dth= 0.0
    sim.MoveDPPourDTheta(ct,l,dp,dth)
    t_curr+= dt
    if t_curr>key_points[-1][0]:  break

def Kickback(ct,l,sim):
  sm= TStateMachine(debug=True)
  sm.EventCallback= ct.SMCallback

  l.dtheta_max= 0.02

  sm.StartState= 'kickback'

  sm.NewState('kickback')
  sm['kickback'].NewAction()
  sm['kickback'].Actions[-1].Condition= lambda: l.sensors.theta<=l.theta_init
  sm['kickback'].Actions[-1].Action= lambda: ( Print('End of pouring') )
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

  ct.RunSM(sm,'flowc3_spline')
  sm.Cleanup()
  return sm.ExitStatus

def Run(ct,*args):
  params= args[0]  #Parameters (dictionary) of this action

  sim= ct.sim
  l= ct.sim_local

  #l.dtheta1= params['dtheta1']
  #l.dtheta2= params['dtheta2']

  while l.sensors.num_src>10:
    dp1= 1.0
    dth1= 5.0
    param= [Rand(-dp1,dp1),Rand(-dp1,dp1),Rand(-dp1,dp1),Rand(-dp1,dp1), Rand(-dth1,dth1),Rand(-dth1,dth1)]
    RunSpline(ct,l,sim,param)
  #Kickback(ct,l,sim)

  return (SUCCESS_CODE, FAILURE_PRECOND, FAILURE_OTHER)[0]

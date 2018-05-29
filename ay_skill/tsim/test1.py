#!/usr/bin/python
from core_tool import *
import std_msgs.msg
import std_srvs.srv
def Help():
  return '''Test of ODE grasping and pouring simulation.
  Usage: tsim.test1'''

def Run(ct,*args):
  l= TContainer(debug=True)

  sim= ct.Load('tsim.core1')

  try:
    sim.SetupServiceProxy(ct,l)
    sim.SetupPubSub(ct,l)

    ct.srvp.ode_resume()
    config= sim.GetConfig(ct)
    print 'Current config:',config

    #Setup config
    config.BallType= 0  #Sphere particles
    #config.BallType= 1  #Box particles

    #Reset to get state for plan
    sim.ResetConfig(ct,config)
    time.sleep(0.1)  #Wait for l.sensors is updated

    #Plan grasp
    #...
    config.GripperHeight= 0.5*l.sensors.p_pour[2]  #Should be in [0,l.sensors.p_pour[2]]

    #Reset again to apply the grasp plan
    sim.ResetConfig(ct,config)
    time.sleep(1.0)  #Wait for reset action is done

    print 'l.sensors:',l.sensors

    #Plan pour
    #...
    p_pour_trg= [l.sensors.x_rcv.position.x-0.1, 0.0, l.sensors.x_rcv.position.z+0.3]
    theta_init= DegToRad(30.0)

    #Move to pour point
    p_pour0= copy.deepcopy(l.sensors.p_pour)
    theta0= l.sensors.theta
    p_pour_msg= std_msgs.msg.Float64MultiArray()
    theta_msg= std_msgs.msg.Float64()
    for tc in FRange1(0.0,1.0,20):
      p_pour_msg.data= (1.0-tc)*Vec(p_pour0) + tc*Vec(p_pour_trg)
      theta_msg.data= (1.0-tc)*theta0 + tc*theta_init
      ct.pub.ode_ppour.publish(p_pour_msg)
      ct.pub.ode_theta.publish(theta_msg)
      sim.SimSleep(ct,l,0.04)

    #Flow control
    theta0= l.sensors.theta
    theta_trg= DegToRad(150.0)
    theta_msg= std_msgs.msg.Float64()
    for tc in FRange1(0.0,1.0,100):
      theta_msg.data= (1.0-tc)*theta0 + tc*theta_trg
      ct.pub.ode_theta.publish(theta_msg)
      sim.SimSleep(ct,l,0.04)

    time.sleep(2.0)  #Observe

  except Exception as e:
    PrintException(e, ' in tsim.test1')

  finally:
    sim.StopPubSub(ct,l)
    ct.srvp.ode_pause()

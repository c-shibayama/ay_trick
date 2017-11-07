#!/usr/bin/python
from core_tool import *
import ay_sim_msgs.msg

def DataBaseDir():
  return '%s/data/' % (os.environ['HOME'])

def VizPP(l,pp,col):
  q= QFromAxisAngle([1.,0.,0.],math.pi*0.5)
  prm= ay_sim_msgs.msg.ODEVizPrimitive()
  prm.type= prm.CYLINDER
  prm.pose.position.x= pp[0]
  prm.pose.position.y= pp[1]-0.18
  prm.pose.position.z= pp[2]
  prm.pose.orientation.x= q[0]
  prm.pose.orientation.y= q[1]
  prm.pose.orientation.z= q[2]
  prm.pose.orientation.w= q[3]
  prm.param= [0.02,0.08]
  prm.color.r= col[0]
  prm.color.g= col[1]
  prm.color.b= col[2]
  prm.color.a= 0.2
  l.user_viz.append(prm)

def VizY2(l,ps_rcv,y2,col):  #NEW_DPLB05
  #x_rcv= GPoseToX(l.sensors.x_rcv)
  #ps_rcv= Get4RcvEdgePoints(l,x_rcv)
  ps_rcv2= [p+dp for p,dp in zip(ps_rcv,y2[:12])]
  prm= ay_sim_msgs.msg.ODEVizPrimitive()
  prm.type= prm.SPHERE
  prm.param= [0.02]
  prm.color.r= col[0]
  prm.color.g= col[1]
  prm.color.b= col[2]
  prm.color.a= 0.2
  for i in range(4):
    prm.pose.position.x= ps_rcv2[3*i+0]
    prm.pose.position.y= ps_rcv2[3*i+1]
    prm.pose.position.z= ps_rcv2[3*i+2]
    l.user_viz.append(copy.deepcopy(prm))

def VizX3(l,pc_rcv,x3,col):  #NEW_DPLB05
  if pc_rcv is None:
    x_rcv= GPoseToX(l.sensors.x_rcv)
    ps_rcv= Get4RcvEdgePoints(l,x_rcv)
    pc_rcv= np.array(ps_rcv).reshape(4,3).mean(axis=0)  #Center of ps_rcv
  term_flow_center= [pc_rcv[0]+2.0*x3[0],
                     pc_rcv[1]+2.0*x3[1] ]
  term_flow_var= x3[2]/5.0
  prm= ay_sim_msgs.msg.ODEVizPrimitive()
  prm.type= prm.LINE
  prm.pose.position.x= term_flow_center[0]
  prm.pose.position.y= term_flow_center[1]
  prm.pose.position.z= 0.0
  prm.param= [0.0,0.0,0.25]
  prm.color.r= col[0]
  prm.color.g= col[1]
  prm.color.b= col[2]
  prm.color.a= 0.2
  l.user_viz.append(prm)
  prm= ay_sim_msgs.msg.ODEVizPrimitive()
  prm.type= prm.CYLINDER
  prm.pose.position.x= term_flow_center[0]
  prm.pose.position.y= term_flow_center[1]
  prm.pose.position.z= 0.0
  prm.param= [max(0.0,term_flow_var), 0.03]
  prm.color.r= col[0]
  prm.color.g= col[1]
  prm.color.b= col[2]
  prm.color.a= 0.1
  l.user_viz.append(prm)

def Get4RcvEdgePoints(l,x_rcv):  #NEW_DPLB05
  rcv_size= l.config.RcvSize
  ps_rcv= sum((Transform(x_rcv,[-0.5*rcv_size[0],-0.5*rcv_size[1],rcv_size[2]]).tolist(),
               Transform(x_rcv,[+0.5*rcv_size[0],-0.5*rcv_size[1],rcv_size[2]]).tolist(),
               Transform(x_rcv,[+0.5*rcv_size[0],+0.5*rcv_size[1],rcv_size[2]]).tolist(),
               Transform(x_rcv,[-0.5*rcv_size[0],+0.5*rcv_size[1],rcv_size[2]]).tolist()),[])
  return ps_rcv

#Observe elements (specified by keys) of XSSA
def ObserveXSSA(l,xs_prev,keys):
  if any(key in keys for key in ('ps_rcv','dps_rcv','lp_pour','lp_pour_trg')):
    ps_rcv= Get4RcvEdgePoints(l, GPoseToX(l.sensors.x_rcv))
  xs= {}
  for key in keys:
    if key=='ps_rcv':  #4 edge point positions (x,y,z)*4 of receiver
      xs[key]= SSA(ps_rcv)
    elif key=='gh_abs':  #Gripper height (absolute value)
      xs[key]= SSA([l.config.GripperHeight])
    elif key=='dps_rcv':  #Displacement of ps_rcv from previous time
      xs[key]= SSA([math.atan(p1-p2) for p1,p2 in
                    zip(ps_rcv, ToList(xs_prev['ps_rcv'].X))])
    elif key=='v_rcv':  #Velocity norm of receiver
      xs[key]= SSA([math.atan(Norm(l.filtered.v_rcv))])
    elif key=='p_pour':  #Pouring axis position (x,y,z)
      xs[key]= SSA([p_pour for p_pour in l.sensors.p_pour])
    elif key=='p_pour_z':  #Pouring axis position (z)
      xs[key]= SSA([l.sensors.p_pour[2]])
    elif key=='lp_pour':  #Pouring axis position (x,y,z) in receiver frame
      xs[key]= SSA([p_pour-pc_rcv for p_pour,pc_rcv in
                    zip(l.sensors.p_pour,
                        np.array(ps_rcv).reshape(4,3).mean(axis=0)  #Center of ps_rcv
                        )])
    elif key=='lp_pour_trg':  #Pouring axis target position (x,y,z) in receiver frame
      xs[key]= SSA([p_pour-pc_rcv for p_pour,pc_rcv in
                    zip(l.p_pour_trg,
                        np.array(ps_rcv).reshape(4,3).mean(axis=0)  #Center of ps_rcv
                        )])
    elif key=='p_flow':  #Flow position (x,y)
      xs[key]= SSA([term_flow_center for term_flow_center in l.filtered.term_flow_center[:2]])
    elif key=='lp_flow':  #Flow position (x,y) in receiver frame
      xs[key]= SSA([math.atan(0.5*(term_flow_center-pc_rcv))
                    for term_flow_center, pc_rcv in
                    zip(l.filtered.term_flow_center[:2],
                        np.array(ps_rcv).reshape(4,3).mean(axis=0)  #Center of ps_rcv
                        )])
    elif key=='lp_flow2':  #Flow position (x,y) in receiver frame (no atan)
      xs[key]= SSA([term_flow_center-pc_rcv
                    for term_flow_center, pc_rcv in
                    zip(l.filtered.term_flow_center[:2],
                        np.array(ps_rcv).reshape(4,3).mean(axis=0)  #Center of ps_rcv
                        )])
    elif key=='lpp_flow':  #Flow position (x,y) relative to previous (before flowctrl) p_pour
      xs[key]= SSA([term_flow_center-p_pour
                    for term_flow_center, p_pour in zip(l.filtered.term_flow_center[:2],ToList(xs_prev['p_pour'].X)[:2])])
    elif key=='flow_var':  #Variance of flow
      xs[key]= SSA([2.0*math.sqrt(l.filtered.term_flow_var)])
    elif key=='a_pour':  #Amount poured in receiver
      xs[key]= SSA([l.filtered.amount])  #==0.0055*l.sensors.num_rcv
    elif key=='a_spill':  #Amount spilled out
      xs[key]= SSA([0.0 if l.filtered.num_spill<1 else -math.atan(0.5*l.filtered.num_spill)])
    elif key=='a_spill2':  #Amount spilled out
      xs[key]= SSA([0.1*l.filtered.num_spill])
    elif key=='a_total':  #Total amount moved from source
      xs[key]= SSA([0.0055*(l.config.BallNum-l.sensors.num_src)])
    elif key=='a_trg':  #Target amount
      xs[key]= SSA([l.amount_trg])
    elif key=='da_pour':  #Amount poured in receiver (displacement)
      xs[key]= SSA([l.filtered.amount - xs_prev['a_pour'].X[0,0]])
    elif key=='da_spill':  #Amount spilled out (displacement)
      xs[key]= SSA([(0.0 if l.filtered.num_spill<1 else -math.atan(0.5*l.filtered.num_spill))
                    - xs_prev['a_spill'].X[0,0]])
    elif key=='da_spill2':  #Amount spilled out (displacement)
      xs[key]= SSA([0.1*l.filtered.num_spill - xs_prev['a_spill2'].X[0,0]])
    elif key=='da_total':  #Total amount moved from source (displacement)
      xs[key]= SSA([0.0055*(l.config.BallNum-l.sensors.num_src) - xs_prev['a_total'].X[0,0]])
    elif key=='da_trg':  #Target amount (displacement)
      if 'amount' in l.filtered:
        xs[key]= SSA([max(0.0, l.amount_trg - l.filtered.amount)])
      else:
        xs[key]= SSA([max(0.0, l.amount_trg)])
    elif key=='size_srcmouth':  #Size of mouth of the source container
      xs[key]= SSA([l.config.SrcSize2H])
    elif key=='material':  #Material property (e.g. viscosity)
      xs[key]= SSA([l.config.ContactBounce, l.config.ContactBounceVel,
                    l.config.ViscosityParam1, l.config.ViscosityMaxDist])
    elif key=='material2':  #Material property (e.g. viscosity)
      xs[key]= SSA([l.config.ContactBounce, l.config.ContactBounceVel,
                    l.config.ViscosityParam1*1.0e6, l.config.ViscosityMaxDist])
  return xs

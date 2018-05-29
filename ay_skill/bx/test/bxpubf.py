#!/usr/bin/python
from core_tool import *
import baxter_core_msgs
def Help():
  return '''Publish end-effector wrench (force, torque) of Baxter.
  Usage: test.bxpubf
    bx.test.bxpubf 'setup'
      Activate.
    bx.test.bxpubf
    bx.test.bxpubf 'clear'
      Deactivate.
'''

def WrenchToList(wrench,l):
  l[0]= wrench.force.x
  l[1]= wrench.force.y
  l[2]= wrench.force.z
  l[3]= wrench.torque.x
  l[4]= wrench.torque.y
  l[5]= wrench.torque.z

def ListToWrench(l,wrench):
  wrench.force.x= l[0]
  wrench.force.y= l[1]
  wrench.force.z= l[2]
  wrench.torque.x= l[3]
  wrench.torque.y= l[4]
  wrench.torque.z= l[5]

def Forwarder(ct,msg,arm,wrench0):
  lwrench= [0]*6
  WrenchToList(msg.wrench, lwrench)
  xw= ct.robot.FK(arm=arm)
  RwT= QToRot(xw[3:]).T
  lwrench[:3]= RwT.dot(lwrench[:3])
  lwrench[3:]= RwT.dot(lwrench[3:])

  msg2= geometry_msgs.msg.WrenchStamped()
  msg2.header= copy.deepcopy(msg.header)
  msg2.header.frame_id= ('right_gripper','left_gripper')[arm]
  ListToWrench(lwrench, msg2.wrench)
  (ct.pub.end_wrench_r,ct.pub.end_wrench_l)[arm].publish(msg2)

  if wrench0[0] is None:
    wrench0[0]= copy.deepcopy(lwrench)
  msg_diff= geometry_msgs.msg.WrenchStamped()
  msg_diff.header= msg2.header
  lwrench_diff= [f-f0 for f,f0 in zip(lwrench,wrench0[0])]
  ListToWrench(lwrench_diff, msg_diff.wrench)
  (ct.pub.end_wrench_diff_r,ct.pub.end_wrench_diff_l)[arm].publish(msg_diff)

def Run(ct,*args):
  if not ct.robot.Is('Baxter'):
    CPrint(4,'This program works only with Baxter.')
    return

  if len(args)>0:
    command= args[0]
    args= args[1:]
  else:
    command= 'clear'
    args= []

  if command=='setup':
    wrench0= [[None],[None]]
    ct.AddPub('end_wrench_r', '/robot/end_wrench_r', geometry_msgs.msg.WrenchStamped)
    ct.AddPub('end_wrench_l', '/robot/end_wrench_l', geometry_msgs.msg.WrenchStamped)
    ct.AddPub('end_wrench_diff_r', '/robot/end_wrench_diff_r', geometry_msgs.msg.WrenchStamped)
    ct.AddPub('end_wrench_diff_l', '/robot/end_wrench_diff_l', geometry_msgs.msg.WrenchStamped)
    ct.AddSub('estate_r', '/robot/limb/right/endpoint_state', baxter_core_msgs.msg.EndpointState,
             lambda msg,ct=ct,wrench0=wrench0: Forwarder(ct,msg,RIGHT,wrench0[RIGHT]))
    ct.AddSub('estate_l', '/robot/limb/left/endpoint_state', baxter_core_msgs.msg.EndpointState,
             lambda msg,ct=ct,wrench0=wrench0: Forwarder(ct,msg,LEFT,wrench0[LEFT]))

  elif command=='clear':
    ct.DelSub('estate_r')
    ct.DelSub('estate_l')
    ct.DelPub('end_wrench_r')
    ct.DelPub('end_wrench_l')
    ct.DelPub('end_wrench_diff_r')
    ct.DelPub('end_wrench_diff_l')

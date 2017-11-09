#!/usr/bin/python
from core_tool import *
def Help():
  return '''Define object properties and store them into attributes (ver. 2).
  Usage: model_loader'''
def Run(ct,*args):
  #NOTE: an attribute named 'l_*' denotes a vector defined on the local frame

  #FIXME: loading from YAML is very slow.  on-demand loading should be required

  if ct.robot.Is('PR2'):
    ct.AddDictAttr(LoadYAML('models/robot/gripper.yaml'))
  elif ct.robot.Is('Baxter'):
    ct.AddDictAttr(LoadYAML('models/robot/gripper_bx.yaml'))
  elif ct.robot.Is('RobotiqNB'):
    ct.AddDictAttr(LoadYAML('models/robot/gripper_rqnb.yaml'))
  elif ct.robot.Is('DxlGripper'):
    ct.AddDictAttr(LoadYAML('models/robot/gripper_dxlg.yaml'))

  #FIXME: the following values should be DEFAULT values
  extra_values= {}
  #Gripper width before grab
  extra_values['g_pre']= 0.09
  #Grab power (max effort):
  extra_values['f_grab']= 50.0
  #Angle to start pouring
  extra_values['pour_start_angle']= 45.0/180.0*math.pi

  def LoadCnt(bid):
    ct.AddDictAttr(bid, LoadYAML('models/obj/%s.yaml'%bid))
    ct.AddDictAttr(bid, extra_values)

  LoadCnt('b50')
  LoadCnt('b51')
  LoadCnt('b52')
  LoadCnt('b53')
  LoadCnt('b54')
  LoadCnt('b55')
  #FIXME:DO NOT OVERWRITE if the following elements already exist.
  ct.SetAttr('memory','flowc','b55','trick_id_means', [0.0,0.0,1.0])
  #ct.SetAttr('memory','flowc','b55','trick_id_sqmeans', [...])
  ct.SetAttr('memory','flowc','b55','shake_axis_theta_means', [math.pi/4.0])
  ct.SetAttr('memory','flowc','b55','shake_axis_theta_std', 0.01)
  LoadCnt('b55b')
  #FIXME:DO NOT OVERWRITE if the following elements already exist.
  ct.SetAttr('memory','flowc','b55b','trick_id_means', [0.0,0.0,1.0])
  #ct.SetAttr('memory','flowc','b55b','trick_id_sqmeans', [...])
  ct.SetAttr('memory','flowc','b55b','shake_axis_theta_means', [math.pi/4.0])
  ct.SetAttr('memory','flowc','b55b','shake_axis_theta_std', 0.01)

  LoadCnt('b56')
  LoadCnt('b57')
  LoadCnt('b58')
  #FIXME:DO NOT OVERWRITE if the following elements already exist.
  ct.SetAttr('memory','flowc','b58','trick_id_means', [0.0,1.0,0.0])

  LoadCnt('b59')
  LoadCnt('b60')
  LoadCnt('b61')
  LoadCnt('b62')
  LoadCnt('b63')
  LoadCnt('b64')
  LoadCnt('b65')
  LoadCnt('b66')

  LoadCnt('b100')
  LoadCnt('b101')
  LoadCnt('b102')

  ct.Run('attr', 'keys')

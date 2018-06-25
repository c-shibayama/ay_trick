#!/usr/bin/python
from core_tool import *
def Help():
  return '''Plan a collision free path to a target end-effector pose.
  Warning: Be careful to moving area of robot.
  Usage: ex.plan_x1
  '''
def Run(ct,*args):
  #Configure parameters for each robot:
  if ct.robot.Is('Baxter'):
    x_table= [0.84,0.0,0.723-0.902, 0.0,0.0,0.0,1.0]  #90.2cm is the height of Baxter pedestal
    table_dim= [0.5,1.8,0.05]
    x_box= [0.7,0.2,-0.05, 0.0,0.0,0.0,1.0]
    box_dim= [0.3,0.3,0.3]
    arm= LEFT
    x_trg= [0.72,-0.12,0.14, 0.6340761624607465,0.7359104869964589,-0.12067513962937848,0.20450106601951204]
    lw_xe= [0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
  elif ct.robot.Is('Motoman'):
    print 'Test this after running: moveq 1'
    x_table= [0.4,0.0,0.30, 0.0,0.0,0.0,1.0]
    table_dim= [0.5,1.8,0.05]
    x_box= [0.4,-0.3,0.45, 0.0,0.0,0.0,1.0]
    box_dim= [0.3,0.3,0.3]
    arm= 0
    x_trg= [0.35, -0.3, 0.7] + list(QFromAxisAngle([0.0,1.0,0.0], math.pi*0.5))
    lw_xe= [0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
  elif ct.robot.Is('Mikata'):
    print 'Test this after running: moveq 1'
    x_table= [0.2,0.0,-0.03, 0.0,0.0,0.0,1.0]
    table_dim= [0.1,0.1,0.01]
    x_box= [0.2,-0.18,0.075, 0.0,0.0,0.0,1.0]
    box_dim= [0.15,0.2,0.12]
    arm= 0
    x_trg= [0.096, -0.09, 0.19, 0,0,0,1]
    lw_xe= [0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
  elif ct.robot.Is('UR'):
    print 'Test this after running: moveq 2'
    x_table= [0.2,0.0,-0.02, 0.0,0.0,0.0,1.0]
    table_dim= [0.5,0.5,0.05]
    x_box= [0.25,-0.2,0.075, 0.0,0.0,0.0,1.0]
    box_dim= [0.15,0.25,0.2]
    arm= 0
    x_trg= [0.22,-0.18,0.22]+list(QFromAxisAngle([0,0,1], -math.pi*0.5))
    lw_xe= [0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
  else:
    raise Exception('ex.plan_x1: Parameters are not configured for:',ct.robot.Name)

  #First we make a planning scene to compute collision.
  #Robot (Baxter) is already involved.

  #Add a table to the planning scene
  table_attr={
      'x': x_table,
      'bound_box': {
        'dim':table_dim,
        'center':[0.0,0.0,-table_dim[2]*0.5, 0.0,0.0,0.0,1.0]
        },
      'shape_primitives': [
        {
          'kind': 'rtpkCuboid',
          'param': [l/2.0 for l in table_dim],  #[half_len_x,half_len_y,half_len_z]
          'pose': [0.0,0.0,-table_dim[2]*0.5, 0.0,0.0,0.0,1.0],
          },
        ],
    }
  #Add table to an internal dictionary:
  ct.SetAttr('table',table_attr)
  #Add table to the planning scene:
  ct.SetAttr(TMP,'scene', LUnion(ct.GetAttrOr([],TMP,'scene'),['table']))

  #Add a box to the planning scene
  box_attr={
      'x': x_box,
      'bound_box': {
        'dim':box_dim,
        'center':[0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
        },
      'shape_primitives': [
        {
          'kind': 'rtpkCuboid',
          'param': [l/2.0 for l in box_dim],  #[half_len_x,half_len_y,half_len_z]
          'pose': [0.0,0.0,0.0, 0.0,0.0,0.0,1.0],
          },
        ],
    }
  #Add box to an internal dictionary:
  ct.SetAttr('box',box_attr)
  #Add box to the planning scene:
  ct.SetAttr(TMP,'scene', LUnion(ct.GetAttrOr([],TMP,'scene'),['box']))

  #Visualize scene:
  ct.Run('viz','')


  #Move to the target with planning a collision free path:
  dt= 5.0  #Duration
  conservative= True  #Ask Yes/No before moving
  ct.Run('adv.move_to_x', x_trg, dt, lw_xe, arm, {}, conservative)


  print 'Clear the planning scene?'
  if AskYesNo():
    #Remove table, box from the planning scene
    ct.SetAttr(TMP,'scene', LDifference(ct.GetAttrOr([],TMP,'scene'),['table','box']))
    #Refresh visualization:
    ct.Run('viz')


#!/usr/bin/python
from core_tool import *
def Help():
  return '''State validity check in a virtual scene while moving the robot.
  Warning: Be careful to moving area of robot.
  Usage: ex.collision1
    Press q to stop the loop.
  '''
def Run(ct,*args):
  #Add a box to the scene
  box_dim= [0.4,1.0,0.1]
  box_attr={
      'x': [0.3,0.,0.0, 0.0,0.0,0.0,1.0],
      'bound_box': {
        'dim': box_dim,
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
  #Add box to the scene:
  ct.SetAttr(TMP,'scene', LUnion(ct.GetAttrOr([],TMP,'scene'),['box']))

  #Setup state-validity-check scene:
  ct.Run('scene', 'make')

  #Visualize scene:
  ct.Run('viz','')

  '''
  #Clear state-validity-check scene:
  ct.Run('scene', 'clear')
  #Remove box from the scene
  ct.SetAttr(TMP,'scene', LDifference(ct.GetAttrOr([],TMP,'scene'),['box']))
  #Refresh visualization:
  ct.Run('viz')
  '''


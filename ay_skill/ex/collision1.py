#!/usr/bin/python
from core_tool import *
def Help():
  return '''State validity check in a virtual scene while moving the robot.
  Warning: Be careful to moving area of robot.
  Usage: ex.collision1
    Press q to stop the loop.
  '''
def Run(ct,*args):
  #Configure parameters for each robot:
  if ct.robot.Is('Motoman'):
    #Move the robot to the initial pose:
    x_trg= lambda t: [0.45,0.2*math.sin(t),0.5]+list(QFromAxisAngle([0,1,0], math.pi*0.5))
    ct.robot.MoveToX(x_trg(0.0), 4.0)
    rospy.sleep(4.0)
    x_box= [0.4,-0.3,0.5, 0.0,0.0,0.0,1.0]
  elif ct.robot.Is('Mikata'):
    #Move the robot to the initial pose:
    x_trg= lambda t: [0.1,0.05*math.sin(t),0.1]+[0,0,0,1]
    ct.robot.MoveToX(x_trg(0.0), 4.0)
    rospy.sleep(4.0)
    x_box= [0.2,-0.23,0.2, 0.0,0.0,0.0,1.0]
  else:
    raise Exception('ex.collision1: Parameters are not configured for:',ct.robot.Name)


  #First we make a virtual scene.

  #Add a box to the scene
  box_dim= [0.3,0.3,0.3]
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
  #Add box to the scene:
  ct.SetAttr(TMP,'scene', LUnion(ct.GetAttrOr([],TMP,'scene'),['box']))

  #Setup state-validity-check scene:
  ct.Run('scene', 'make')

  #Visualize scene:
  ct.Run('viz','')

  #Visualize contacts:
  viz_c= TSimpleVisualizer(rospy.Duration(1.0), name_space='visualizer_contacts')
  viz_c.viz_frame= ct.robot.BaseFrame

  time0= rospy.Time.now()
  kbhit= TKBHit()
  try:
    while True:
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':
          break;
      else:
        break

      t= (rospy.Time.now()-time0).to_sec()
      ct.robot.MoveToX(x_trg(t), 0.1)

      vs,res1= ct.Run('scene','isvalidq',ct.robot.Arm,[],ct.robot.Q())
      if not vs:  print '{t}: Arm-{arm} is in contact'.format(t=t, arm=ct.robot.ArmStr())
      viz_c.AddContacts(res1.contacts, scale=[0.05])

      rospy.sleep(0.1)
      viz_c.DeleteAllMarkers()

  finally:
    kbhit.Deactivate()


  #Clear state-validity-check scene:
  ct.Run('scene', 'clear')

  #Remove box from the scene
  ct.SetAttr(TMP,'scene', LDifference(ct.GetAttrOr([],TMP,'scene'),['box']))
  #Refresh visualization:
  ct.Run('viz')


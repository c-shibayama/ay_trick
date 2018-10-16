#!/usr/bin/python
from core_tool import *
def Help():
  return '''State validity check in a virtual scene while moving the robot with grasping an object.
  Warning: Be careful to moving area of robot.
  Usage: ex.collision3
    Press q to stop the loop.
  '''
def Run(ct,*args):
  arm= ct.robot.Arm
  f_acc= 1.0
  #Configure parameters for each robot:
  if ct.robot.Is('Motoman'):
    #Move the robot to the initial pose:
    x_trg= lambda t: [0.45,0.2*math.sin(t),0.5]+list(QFromAxisAngle([0,1,0], math.pi*0.5))
    ct.robot.MoveToX(x_trg(0.0), 4.0*f_acc, blocking=True, arm=arm)
    x_box= [0.5,-0.3,0.5, 0.0,0.0,0.0,1.0]
    x_obj= [0.55,0.2,0.5, 0.0,0.0,0.0,1.0]
    lw_xe= ct.GetAttr('wrist_r','lx')
  else:
    raise Exception('ex.collision3: Parameters are not configured for:',ct.robot.Name)


  #First we make a virtual scene.

  #Add a box to the scene
  box_dim= [0.4,0.3,0.3]
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

  #Add an object to the scene
  obj_dim= [0.09,0.09,0.13]
  obj_attr={
      'x': x_obj,
      'bound_box': {
        'dim':obj_dim,
        'center':[0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
        },
      'shape_primitives': [
        {
          'kind': 'rtpkCylinder',
          'param': [0.04, 0.12],  #[radius,height]
          'pose': [0.0,0.0,0.0, 0.0,0.0,0.0,1.0],
          },
        ],
    }
  #Add object to an internal dictionary:
  ct.SetAttr('obj',obj_attr)
  #Add object to the scene:
  ct.SetAttr(TMP,'scene', LUnion(ct.GetAttrOr([],TMP,'scene'),['obj']))

  #Move to grasp the object
  ct.robot.MoveToX(x_obj, 3.0*f_acc, x_ext=lw_xe, blocking=True, arm=arm)
  #Virtually grasp the object
  ct.Run('scene','grab',arm,'obj')

  #Setup state-validity-check scene:
  ct.Run('scene', 'make')
  rospy.sleep(0.3)

  #Visualize scene:
  ct.Run('viz','')

  #Move to the initial pose
  ct.robot.MoveToX(x_trg(0.0), 3.0*f_acc, blocking=True, arm=arm)

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
      ct.robot.MoveToX(x_trg(t), 0.07)

      vs,res1= ct.Run('scene','isvalidq',arm,[],ct.robot.Q(arm))
      if not vs:  print '{t}: Arm-{arm} is in contact'.format(t=t, arm=ct.robot.ArmStr(arm))
      viz_c.AddContacts(res1.contacts, scale=[0.05])
      #print res1.contacts

      rospy.sleep(0.1)
      viz_c.DeleteAllMarkers()

  finally:
    kbhit.Deactivate()

  #Clear state-validity-check scene:
  ct.Run('scene', 'clear')

  #Virtually release the object
  ct.Run('scene','release','obj')

  #Move to the initial pose
  ct.robot.MoveToX(x_trg(0.0), 3.0*f_acc, blocking=True, arm=arm)

  ##Remove box from the scene
  #ct.SetAttr(TMP,'scene', LDifference(ct.GetAttrOr([],TMP,'scene'),['box']))
  ##Refresh visualization:
  #ct.Run('viz')


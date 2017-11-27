#!/usr/bin/python
from core_tool import *
def Help():
  return '''State validity check in a virtual scene where the object is moving.
  Usage: ex.collision2
    Press q to stop the loop.
  '''
def Run(ct,*args):
  #First we make a virtual scene.

  #Add a box to the scene
  box_dim= [0.3,0.3,0.3]
  x_box= lambda t: [0.3*math.sin(t),0.3*math.sin(2.0*t),0.5]+[0.0,0.0,0.0,1.0]
  box_attr={
      'x': x_box(0.0),
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

  #Suppress messages in scene.
  ct.Run('displevel','quiet')

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

      #Setup state-validity-check scene:
      ct.Run('scene', 'make')

      t= (rospy.Time.now()-time0).to_sec()
      box_attr['x']= x_box(t)

      vs,res1= ct.Run('scene','isvalidq',ct.robot.Arm,[],ct.robot.Q())
      if not vs:  print '{t}: Arm-{arm} is in contact'.format(t=t, arm=ct.robot.ArmStr())
      viz_c.AddContacts(res1.contacts, scale=[0.05])

      #Clear state-validity-check scene:
      ct.Run('scene', 'clear')

      rospy.sleep(0.1)
      viz_c.DeleteAllMarkers()

  finally:
    kbhit.Deactivate()


  ct.Run('displevel','normal')

  #Remove box from the scene
  ct.SetAttr(TMP,'scene', LDifference(ct.GetAttrOr([],TMP,'scene'),['box']))
  #Refresh visualization:
  ct.Run('viz')


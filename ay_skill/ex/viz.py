#!/usr/bin/python
from core_tool import *
def Help():
  return '''Examples of TSimpleVisualizer.
  Usage: ex.viz [FRAME]
    FRAME: Frame of visualization (default: ct.robot.BaseFrame).
  '''

def HelpKeyOp():
  print '''Keyboard operation:
  q: Quit.
  h: Help.
  d: Delete all (DeleteAllMarkers).
  a: Add an arrow (AddArrow).
  w: Add a list of arrows (AddArrowList).
  c: Add a cube (AddCube).
  s: Add a sphere (AddSphere).
  y: Add a cylinder (AddCylinder).
  x: Add a cylinder (AddCylinderX).
  p: Add points (AddPoints).
  o: Add a coordinate system (AddCoord).
  9: Add a coordinate system (AddCoordC).
  0: Add a list of coordinate systems (AddArrowList).
  l: Add a polygon (AddPolygon).
  k: Add a list of lines (AddLineList).
  '''

def Run(ct,*args):
  frame= args[0] if len(args)>0 else ct.robot.BaseFrame
  HelpKeyOp()

  viz= TSimpleVisualizer(rospy.Duration(), name_space='visualizer_test', frame=frame)
  mid= 0
  alpha= 0.8

  RandX= lambda: np.random.uniform(-0.5,0.5,3).tolist() + QFromAxisAngle(np.random.uniform(0,1,3),np.random.uniform(-np.pi,np.pi)).tolist()
  RandXL= lambda N: [RandX() for i in range(N)]
  RandP= lambda: np.random.uniform(-0.5,0.5,3).tolist()
  RandCol= lambda: viz.ICol(np.random.randint(6))
  RandColL= lambda N: [viz.ICol(np.random.randint(6)) for i in range(N)]

  with TKBHit() as kbhit:
    while not rospy.is_shutdown():
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':  break
        elif key=='h':  HelpKeyOp()
        elif key=='d':
          viz.DeleteAllMarkers()
          viz.Reset()
          mid= 0
        elif key=='a':
          scale= (lambda a,b: [a,b,b])(np.random.uniform(0,0.5),np.random.uniform(0,0.01))
          mid= viz.AddArrow(x=RandX(), scale=scale, rgb=RandCol(), alpha=alpha, mid=mid)
        elif key=='w':
          N= 10
          scale= [np.random.uniform(0,0.5),np.random.uniform(0,0.01)]
          mid= viz.AddArrowList(x_list=RandXL(N), scale=scale, rgb=RandColL(N), alpha=alpha, mid=mid)
        elif key=='c':
          scale= np.random.uniform(0,0.5,3)
          mid= viz.AddCube(x=RandX(), scale=scale, rgb=RandCol(), alpha=alpha, mid=mid)
        elif key=='s':
          scale= (lambda r: [r,r,r])(np.random.uniform(0,0.5))
          mid= viz.AddSphere(p=RandP(), scale=scale, rgb=RandCol(), alpha=alpha, mid=mid)
        elif key=='y':
          diameter= np.random.uniform(0,0.5)
          mid= viz.AddCylinder(p1=RandP(), p2=RandP(), diameter=diameter, rgb=RandCol(), alpha=alpha, mid=mid)
        elif key=='x':
          axis= ('x','y','z')[np.random.randint(3)]
          diameter= np.random.uniform(0,0.5)
          l1= np.random.uniform(0,0.5)
          l2= np.random.uniform(l1,1.0)
          mid= viz.AddCylinderX(x=RandX(), axis=axis, diameter=diameter, l1=l1, l2=l2, rgb=RandCol(), alpha=alpha, mid=mid)
        elif key=='p':
          N= 20
          points= [RandP() for i in range(N)]
          scale= np.random.uniform(0,0.1,2)
          mid= viz.AddPoints(points=points, scale=scale, rgb=RandColL(N), alpha=alpha, mid=mid)
        elif key=='o':
          scale= np.random.uniform(0,0.1,2)
          mid= viz.AddCoord(x=RandX(), scale=scale, alpha=alpha, mid=mid)
        elif key=='9':
          scale= np.random.uniform(0,0.1,2)
          mid= viz.AddCoordC(x=RandX(), scale=scale, alpha=alpha, mid=mid)
        elif key=='0':
          scale= [np.random.uniform(0,0.5),np.random.uniform(0,0.01)]
          x_list= RandXL(10)
          mid= viz.AddArrowList(x_list=x_list, axis='x', scale=scale, rgb=viz.ICol(0), alpha=alpha, mid=mid)
          mid= viz.AddArrowList(x_list=x_list, axis='y', scale=scale, rgb=viz.ICol(1), alpha=alpha, mid=mid)
          mid= viz.AddArrowList(x_list=x_list, axis='z', scale=scale, rgb=viz.ICol(2), alpha=alpha, mid=mid)
        elif key=='l':
          N= 20
          points= np.random.uniform(-0.5,0.5,(N,3)).tolist()
          scale= np.random.uniform(0,0.1,1)
          mid= viz.AddPolygon(points, scale=scale, rgb=RandColL(N), alpha=alpha, mid=mid)
        elif key=='k':
          N= 20
          points= np.random.uniform(-0.5,0.5,(N,3)).tolist()
          scale= np.random.uniform(0,0.1,1)
          mid= viz.AddLineList(points, scale=scale, rgb=RandColL(N), alpha=alpha, mid=mid)
      else:  break
      rospy.sleep(200.0e-3)


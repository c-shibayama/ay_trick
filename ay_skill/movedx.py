#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm in target end-effector pose displacement.
  Warning: Be careful to moving area of robot.
  Usage: movedx DPOSE [, DURATION]
    DPOSE: Pose-displacement vector, which can be:
      [dx,dy,dz]:  Position displacement.
      [dx,dy,dz, wx,wy,wz]:  Position and orientation displacement.
          NOTE: [wx,wy,wz] == angle*axis (axis: normalized 3D vector of rotation axis, angle: angle in radian).
    DURATION: Duration of motion.  Default: 4.0 '''
def Run(ct,*args):
  dx= args[0]
  dt= args[1] if len(args)>1 else 4.0
  x_trg= np.array(ct.robot.FK())
  if len(dx)==3:
    x_trg[:3]+= dx
  elif len(dx)==6:
    x_trg[:3]+= dx[:3]
    x_trg[3:]= MultiplyQ(QFromAxisAngle(dx[3:],la.norm(dx[3:])), x_trg[3:])
  else:  raise Exception('movedx: Unacceptable DPOSE.')
  print 'Move to x:',x_trg
  ct.robot.MoveToX(x_trg, dt)

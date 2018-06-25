#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to a given joint positions.
  Warning: Be careful to moving area of robot.
  Usage: moveq JOINT_ANGLES [, DURATION]
    JOINT_ANGLES: Joint angle vector.
      JOINT_ANGLES can be a number (0,1,...).
      In that case, stored joint angles (key poses) are used.
    DURATION: Duration of motion.  Default: 4.0 '''
def Run(ct,*args):
  q_trg= args[0]
  dt= args[1] if len(args)>1 else 4.0
  if isinstance(q_trg,int):
    if ct.robot.Is('PR2'):
      keyposes= [[],[]]
      keyposes[RIGHT].append([-1.5758421026969418, 1.2968352230407523, -1.6520923310211921, -2.095963566248973, 10.512690320637843, -1.469029183486648, 2.37512293699])
      keyposes[LEFT].append([0.48729783262306103, 0.5076493497258632, 1.8506637154437104, -2.1147837587288314, -3.0056602498891145, -1.8564307782457792, 1.4902347755205518])
    elif ct.robot.Is('Baxter'):
      keyposes= [[],[]]
      keyposes[RIGHT].append([-0.7090826183898926, -0.15071361223754884, -0.031063110919189455, 2.0229371617126466, -2.3151605014709475, -0.5836796891235352, 0.4237621921691895])
      keyposes[LEFT].append([-0.15263108822021484, -0.10737865502929689, -0.36777189347534184, 2.1590779564819336, 0.4782185100769043, -1.3487526062072754, 0.5418787127014161])
    elif ct.robot.Is('Motoman'):
      keyposes= [[]]
      keyposes[0].append([0.0]*7)
      #NOTE: Generated by: IK([0.45,0.0,0.5]+list(QFromAxisAngle([0,1,0], math.pi*0.5)))
      keyposes[0].append([-0.02225494707637879, 0.027604753814144237, 0.02256845844164128, -2.2001560115435073, -0.00047772651727832574, 0.6569580325147487, 0.0010119170182285682])
      keyposes[0].append([0.21, -0.59, 0.3, -1.46, 0.35, -0.68, 0.31])
    elif ct.robot.Is('Mikata'):
      keyposes= [[]]
      keyposes[0].append([0.0]*4)
      #NOTE: Generated by: IK([0.1,0.0,0.1, 0,0,0,1])
      keyposes[0].append([1.914316229955623e-06, -0.04694073844180478, 1.0316071071894901, -0.9846663687476853])
      keyposes[0].append([0, 0, 1, -1.3])
    elif ct.robot.Is('UR'):
      keyposes= [[]]
      keyposes[0].append([0.0, -math.pi*0.5, 0.0, -math.pi*0.5, 0.0, 0.0])
      #keyposes[0].append([0.0, -2.1, -2.1, -0.57, 0.0, math.pi])
      keyposes[0].append([0.0, -1.04, 2.1, -math.pi*0.5-1.06, 0.0, -math.pi])
      #keyposes[0].append([0.0, -2.1, -2.1, -2.1, -math.pi*0.5, math.pi])
      keyposes[0].append([0.0, -1.04, 2.1, -1.06, math.pi*0.5, -math.pi])
      #keyposes[0].append([0.0]*4)
    q_trg= keyposes[ct.robot.Arm][q_trg]
  assert(len(q_trg)==ct.robot.DoF())
  print 'Move to q:',q_trg
  ct.robot.MoveToQ(q_trg, dt)

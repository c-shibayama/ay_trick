#!/usr/bin/python
from core_tool import *
def Help():
  return '''Automatic put motion: stop putting when it detects force change or slip.
  Usage:
    fv.put 'on' [, ARM [, OPTIONS]]
      Turn on a putting thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
      OPTIONS: Options of PuttingLoop (see PuttingLoopDefaultOptions).
    fv.put 'off' [, ARM]
      Stop the putting thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.put
    fv.put 'clear'
      Stop all threads. Equivalent to:
        fv.put 'off' LEFT
        fv.put 'off' RIGHT'''

def PuttingLoopDefaultOptions(ct):
  #NOTE: We inherit the options of fv.openif
  options= ct.Load('fv.openif').OpeningLoopDefaultOptions()
  options['z_move']= 0.01  #z-distance to move.
  options['vz']= -0.004  #z-velocity (m/s).  WARNING: We assume slow motion which the robot can stop suddenly.
  return options

def PuttingLoop(th_info, ct, arm, options):
  ct.Run('fv.ctrl_params')
  fv_data= ct.GetAttr(TMP,'fv'+ct.robot.ArmStrS(arm))

  openif_options= options
  ct.Run('fv.openif','on',arm,openif_options)

  options['log']['active']= True
  if options['z_move']<0.0:  options['z_move']= abs(options['z_move'])

  try:
    velctrl= ct.Run('velctrl',arm)

    x_trg= ct.robot.FK(arm=arm)
    z0= x_trg[2]
    last_dq= [0.0]*ct.robot.DoF(arm)

    cmd= None
    while not rospy.is_shutdown():
      x1= ct.robot.FK(arm=arm)
      J1= ct.robot.J(arm=arm)
      #dq1= ct.robot.DQ(arm=arm)
      dq1= last_dq
      '''NOTE
      We use last-dq (target velocities) rather than current actual velocities
      in order to avoid oscillation.
      '''
      dx1= ToList(J1 * MCVec(dq1))

      if 'opened' in openif_options['log'] and openif_options['log']['opened']:
        velctrl.Step([0.0]*ct.robot.DoF(arm))
        break

      if abs(x1[2]-z0)>=options['z_move']:
        velctrl.Step([0.0]*ct.robot.DoF(arm))
        break

      x_trg[2]+= options['vz']*velctrl.TimeStep()

      kp= [1.0,1.0, 4.0,  1.0,1.0,1.0]  #Position gain.
      kd= [0.01,0.01, 0.03,  0.01,0.01,0.01]  #Velocity gain.
      x_err= DiffX(x1,x_trg)
      vx= [kp[d]*x_err[d] - kd[d]*dx1[d] for d in range(6)]

      if ct.robot.DoF(arm=arm)>=6:
        dq= ToList(la.pinv(J1)*MCVec(vx))
      else:  #e.g. Mikata Arm
        W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
        dq= ToList(la.pinv(W*J1)*W*MCVec(vx))
      velctrl.Step(dq)

      last_dq= dq

    print ''

  finally:
    velctrl.Step([0.0]*ct.robot.DoF(arm))
    velctrl.Finish()
    ct.Run('fv.openif','off',arm)
    options['log']['active']= False


def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    user_options= args[1] if len(args)>1 else {}

    options= PuttingLoopDefaultOptions(ct)
    InsertDict(options, user_options)
    if 'log' in user_options:  options['log']= user_options['log']

    if 'vs_put'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'vs_put'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    ct.Run('fv.grasp','off',arm)
    ct.Run('fv.hold','off',arm)

    CPrint(1,'Turn on:','vs_put'+LRToStrS(arm))
    ct.thread_manager.Add(name='vs_put'+LRToStrS(arm), target=lambda th_info: PuttingLoop(th_info,ct,arm,options))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    CPrint(2,'Turn off:','vs_put'+LRToStrS(arm))
    ct.thread_manager.Stop(name='vs_put'+LRToStrS(arm))

  elif command=='clear':
    CPrint(2,'Turn off:','vs_put'+LRToStrS(RIGHT))
    CPrint(2,'Turn off:','vs_put'+LRToStrS(LEFT))
    ct.thread_manager.Stop(name='vs_put'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='vs_put'+LRToStrS(LEFT))

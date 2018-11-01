#!/usr/bin/python
from core_tool import *
import tf
def Help():
  return '''Robot initialization utility.
  Usage: robot ROBOT_NAME [, OPT1]
    Available ROBOT_NAME:
      'NoRobot',
      'pr2','PR2',
      'pr2s','PR2_SIM',
      'bx','Baxter',
      'bxs','Baxter_SIM',
      'bxn','BaxterN',
      'rq','RobotiqNB',
      'dxlg','DxlGripper',
      'moto','Motoman',
      'motos','Motoman_SIM',
      'mikata','Mikata',
      'mikatas','Mikata_SIM',
      'mikata2','Mikata2',
      'cx7','CraneX7',
      'ur','UR',
      'urs','UR_SIM',
      'urdxlg','URDxlG',
      'urdxlgs','URDxlG_SIM',
    If ROBOT_NAME is omitted, we assume 'NoRobot'.

    Definition of OPT1 depends on ROBOT_NAME.
      'dxlg','mikata','urdxlg': Device name (default='/dev/ttyUSB0')

  '''
def Run(ct,*args):
  robot= args[0] if len(args)>0 else 'NoRobot'

  alias={
      'pr2':'PR2',
      'pr2s':'PR2_SIM',
      'bx':'Baxter',
      'bxs':'Baxter_SIM',
      'bxn':'BaxterN',
      'rq':'RobotiqNB',
      'dxlg':'DxlGripper',
      'moto':'Motoman',
      'motos':'Motoman_SIM',
      'mikata':'Mikata',
      'mikatas':'Mikata_SIM',
      'mikata2':'Mikata2',
      'cx7':'CraneX7',
      'ur':'UR',
      'urs':'UR_SIM',
      'urdxlg':'URDxlG',
      'urdxlgs':'URDxlG_SIM',
    }
  if robot in alias:  robot= alias[robot]

  print 'Setup robot for',robot

  if ct.state_validity_checker is not None:
    del ct.state_validity_checker
    ct.state_validity_checker= None
  if ct.robot is not None and not ct.robot.Is('NoRobot'):
    ct.Run('fv.fv','clear')

    ct.robot.Cleanup()
    del ct.robot
    ct.robot= None

  if   robot in ('PR2','PR2_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_pr2')
    ct.robot= mod.TRobotPR2()

  elif robot in ('Baxter','Baxter_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_bxtr')
    ct.robot= mod.TRobotBaxter(is_sim=(robot=='Baxter_SIM'))

  elif robot in ('BaxterN',):
    mod= SmartImportReload('ay_py.ros.rbt_bxtrN')
    ct.robot= mod.TRobotBaxterN()

  elif robot in ('RobotiqNB',):
    mod= SmartImportReload('ay_py.ros.rbt_rqnb')
    ct.robot= mod.TRobotRobotiqNB()

  elif robot in ('DxlGripper',):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_dxlg')
    #serial_dev= os.environ['DXLG_DEV'] if 'DXLG_DEV' in os.environ else '/dev/ttyUSB0'
    ct.robot= mod.TRobotDxlGripper(dev=serial_dev)

  elif robot in ('Motoman','Motoman_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_moto')
    ct.robot= mod.TRobotMotoman(is_sim=(robot=='Motoman_SIM'))

  elif robot in ('Mikata','Mikata_SIM'):
    if robot=='Mikata':
      CPrint(4,'''We recommend to use Mikata2 (mikata2), where you need to launch:
$ roslaunch ay_util mikata_rot_real2.launch  # Or, mikata_real2.launch
Do you want to abort?''')
      if AskYesNo():  return
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_mikata')
    ct.robot= mod.TRobotMikata(dev=serial_dev,is_sim=(robot=='Mikata_SIM'))

  elif robot in ('Mikata2',):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_mikata2')
    ct.robot= mod.TRobotMikata2()

  elif robot in ('CraneX7',):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_cranex7')
    ct.robot= mod.TRobotCraneX7()

  elif robot in ('UR','UR_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_ur')
    ct.robot= mod.TRobotUR(is_sim=(robot=='UR_SIM'))

  elif robot in ('URDxlG','URDxlG_SIM'):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_urdxlg')
    ct.robot= mod.TRobotURDxlG(is_sim=(robot=='URDxlG_SIM'),dev=serial_dev)

  elif robot=='NoRobot':
    ct.robot= TFakeRobot()
  else:
    raise Exception('Unknown robot: %s'%robot)

  if robot=='NoRobot':  return

  ct.br= tf.TransformBroadcaster()

  if any((ct.robot.Is('PR2'),ct.robot.Is('Baxter'),ct.robot.Is('Motoman'),ct.robot.Is('Mikata'),ct.robot.Is('UR'),ct.robot.Is('URDxlG'))):
    ct.state_validity_checker= TStateValidityCheckerMI()
  else:
    ct.state_validity_checker= None

  res= []
  ra= lambda r: res.append(r)

  ra(ct.robot.Init())
  if ct.state_validity_checker is not None:
    ra(ct.state_validity_checker.Init(ct.robot))

  if False in res:
    CPrint(4, 'Failed to setup robot:',robot)

  if robot in ('PR2','Baxter','BaxterN','Motoman','Mikata','Mikata2','CraneX7','UR','URDxlG'):
    ct.SetAttr('environment', 'real')
  elif robot in ('PR2_SIM','Baxter_SIM','Motoman_SIM','Mikata_SIM','UR_SIM','URDxlG_SIM'):
    ct.SetAttr('environment', 'sim')

  #ct.Run('model_loader')

  model_dir= os.path.dirname(__file__)+'/data'
  if ct.robot.Is('PR2'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_pr2.yaml'))
  elif ct.robot.Is('Baxter'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_bx.yaml'))
  elif ct.robot.Is('RobotiqNB'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_rqnb.yaml'))
  elif ct.robot.Is('DxlGripper'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_dxlg.yaml'))
  elif ct.robot.Is('Motoman'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_moto.yaml'))
  elif ct.robot.Is('Mikata'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_mikata.yaml'))
  elif ct.robot.Is('URDxlG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_urdxlg.yaml'))
  ct.SetAttr('default_frame', ct.robot.BaseFrame)

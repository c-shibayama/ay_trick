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
      'thg','RHP12RNGripper',
      'moto','Motoman',
      'motos','Motoman_SIM',
      'mikata','Mikata',
      'mikatas','Mikata_SIM',
      'mikata2','Mikata2',
      'mikata6','Mikata6',
      'mikata6s','Mikata6_SIM',
      'cx7','CraneX7',
      'ur3','UR3',
      'ur3s','UR3_SIM',
      'ur3dxlg','UR3DxlG',
      'ur3dxlgs','UR3DxlG_SIM',
      'ur3thg','UR3ThG',
      'ur3thgs','UR3ThG_SIM',
      'ur3e','UR3e',
      'ur3es','UR3e_SIM',
      'ur3ethg','UR3eThG',
      'ur3ethgs','UR3eThG_SIM',
      'ur5e','UR5e',
      'ur5es','UR5e_SIM',
      'ur5ethg','UR5eThG',
      'ur5ethgs','UR5eThG_SIM',
    If ROBOT_NAME is omitted, we assume 'NoRobot'.

    Definition of OPT1 depends on ROBOT_NAME.
      'dxlg','mikata','ur*dxlg','ur*thg': Device name (default='/dev/ttyUSB0')

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
      'thg':'RHP12RNGripper',
      'moto':'Motoman',
      'motos':'Motoman_SIM',
      'mikata':'Mikata',
      'mikatas':'Mikata_SIM',
      'mikata2':'Mikata2',
      'mikata6':'Mikata6',
      'mikata6s':'Mikata6_SIM',
      'cx7':'CraneX7',
      'ur3':'UR3',
      'ur3s':'UR3_SIM',
      'ur3dxlg':'UR3DxlG',
      'ur3dxlgs':'UR3DxlG_SIM',
      'ur3thg':'UR3ThG',
      'ur3thgs':'UR3ThG_SIM',
      'ur3e':'UR3e',
      'ur3es':'UR3e_SIM',
      'ur3ethg':'UR3eThG',
      'ur3ethgs':'UR3eThG_SIM',
      'ur5e':'UR5e',
      'ur5es':'UR5e_SIM',
      'ur5ethg':'UR5eThG',
      'ur5ethgs':'UR5eThG_SIM',
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

  elif robot in ('RHP12RNGripper',):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_rhp12rn')
    ct.robot= mod.TRobotRHP12RNGripper(dev=serial_dev)

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

  elif robot in ('Mikata6',):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_mikata6')
    ct.robot= mod.TRobotMikata6()

  elif robot in ('CraneX7',):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_cranex7')
    ct.robot= mod.TRobotCraneX7()

  elif robot in ('UR3','UR3_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_ur')
    ct.robot= mod.TRobotUR(name='UR3',ur_series='CB',is_sim=(robot=='UR3_SIM'))

  elif robot in ('UR3DxlG','UR3DxlG_SIM'):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_urdxlg')
    ct.robot= mod.TRobotURDxlG(name='UR3DxlG',ur_series='CB',is_sim=(robot=='UR3DxlG_SIM'),dev=serial_dev)

  elif robot in ('UR3ThG','UR3ThG_SIM'):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_urthg')
    ct.robot= mod.TRobotURThG(name='UR3ThG',ur_series='CB',is_sim=(robot=='UR3ThG_SIM'),dev=serial_dev)

  elif robot in ('UR3e','UR3e_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_ur')
    ct.robot= mod.TRobotUR(name='UR3e',ur_series='E',is_sim=(robot=='UR3e_SIM'))

  elif robot in ('UR3eThG','UR3eThG_SIM'):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_urthg')
    ct.robot= mod.TRobotURThG(name='UR3eThG',ur_series='E',is_sim=(robot=='UR3eThG_SIM'),dev=serial_dev)

  elif robot in ('UR5e','UR5e_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_ur')
    ct.robot= mod.TRobotUR(name='UR5e',ur_series='E',is_sim=(robot=='UR5e_SIM'))

  elif robot in ('UR5eThG','UR5eThG_SIM'):
    serial_dev= args[1] if len(args)>1 else '/dev/ttyUSB0'
    mod= SmartImportReload('ay_py.ros.rbt_urthg')
    ct.robot= mod.TRobotURThG(name='UR5eThG',ur_series='E',is_sim=(robot=='UR5eThG_SIM'),dev=serial_dev)

  elif robot=='NoRobot':
    ct.robot= TFakeRobot()
  else:
    raise Exception('Unknown robot: %s'%robot)

  if robot=='NoRobot' or ct.robot is None:  return

  ct.br= tf.TransformBroadcaster()

  robots_with_state_validity_checker= ('PR2','Baxter','Motoman','Mikata','UR')
  if any([ct.robot.Is(rbt) for rbt in robots_with_state_validity_checker]):
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

  # Is this attribute ('environment') used?? --> OBSOLETE_CANDIDATE
  if not ct.robot.Is('sim'):
    ct.SetAttr('environment', 'real')
  elif ct.robot.Is('sim'):
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
  elif ct.robot.Is('RHP12RNGripper'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_thg.yaml'))
  elif ct.robot.Is('Motoman'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_moto.yaml'))
  elif ct.robot.Is('Mikata'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_mikata.yaml'))
  elif ct.robot.Is('UR3DxlG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3dxlg.yaml'))
  elif ct.robot.Is('UR3ThG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3thg.yaml'))
  elif ct.robot.Is('UR3eThG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3ethg.yaml'))
  elif ct.robot.Is('UR5eThG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur5ethg.yaml'))
  ct.SetAttr('default_frame', ct.robot.BaseFrame)

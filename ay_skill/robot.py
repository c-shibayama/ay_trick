#!/usr/bin/python
from core_tool import *
def Help():
  return '''Robot initialization utility.
  Usage: robot ROBOT_NAME
    Available ROBOT_NAME:
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
    }
  if robot in alias:  robot= alias[robot]

  print 'Setup robot for',robot

  if ct.state_validity_checker is not None:
    del ct.state_validity_checker
    ct.state_validity_checker= None
  if ct.robot is not None and not ct.robot.Is('NoRobot'):
    ct.Run('fv.finger3','clear')

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
    mod= SmartImportReload('ay_py.ros.rbt_dxlg')
    serial_dev= os.environ['DXLG_DEV'] if 'DXLG_DEV' in os.environ else '/dev/ttyUSB0'
    ct.robot= mod.TRobotDxlGripper(dev=serial_dev)

  elif robot in ('Motoman','Motoman_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_moto')
    ct.robot= mod.TRobotMotoman(is_sim=(robot=='Motoman_SIM'))

  elif robot in ('Mikata','Mikata_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_mikata')
    ct.robot= mod.TRobotMikata(dev='/dev/ttyUSB0',is_sim=(robot=='Mikata_SIM'))

  elif robot=='NoRobot':
    ct.robot= TFakeRobot()
  else:
    raise Exception('Unknown robot: %s'%robot)

  if robot=='NoRobot':  return

  if any((ct.robot.Is('PR2'),ct.robot.Is('Baxter'),ct.robot.Is('Motoman'),ct.robot.Is('Mikata'))):
    ct.state_validity_checker= TStateValidityCheckerMI()

  res= []
  ra= lambda r: res.append(r)

  ra(ct.robot.Init())
  if any((ct.robot.Is('PR2'),ct.robot.Is('Baxter'),ct.robot.Is('Motoman'),ct.robot.Is('Mikata'))):
    ra(ct.state_validity_checker.Init(ct.robot))

  if False in res:
    CPrint(4, 'Failed to setup robot:',robot)

  if robot in ('PR2','Baxter','BaxterN','Motoman','Mikata'):
    ct.SetAttr('environment', 'real')
  elif robot in ('PR2_SIM','Baxter_SIM','Motoman_SIM','Mikata_SIM'):
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
    #ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_moto.yaml'))
    pass
  elif ct.robot.Is('Mikata'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_mikata.yaml'))
  ct.SetAttr('default_frame', ct.robot.BaseFrame)

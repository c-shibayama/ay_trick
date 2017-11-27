#!/usr/bin/python
from core_tool import *
def Help():
  return '''Default script executed at the beginning.
    The Main function defined below is executed as a thread,
    which is registered as "_default".
  Usage: (AUTOMATICALLY EXECUTED)'''

import tf

def Main(ct,th_info):
  ct.br= tf.TransformBroadcaster()

  if ROS_ROBOT=='NoRobot':
    ct.robot= TFakeRobot()
    return

  if ROS_ROBOT in ('PR2','PR2_SIM',
                   'Baxter','Baxter_SIM','BaxterN',
                   'RobotiqNB','DxlGripper',
                   'Motoman','Motoman_SIM'):
    print 'Setup robot for',ROS_ROBOT
    if   ROS_ROBOT in ('PR2','PR2_SIM'):  ct.robot= TRobotPR2()
    elif ROS_ROBOT in ('Baxter','Baxter_SIM'):  ct.robot= TRobotBaxter()
    elif ROS_ROBOT in ('BaxterN',):  ct.robot= TRobotBaxterN()
    elif ROS_ROBOT in ('RobotiqNB',):  ct.robot= TRobotRobotiqNB()
    elif ROS_ROBOT in ('DxlGripper',):  ct.robot= TRobotDxlGripper(dev=os.environ['DXLG_DEV'] if 'DXLG_DEV' in os.environ else '/dev/ttyUSB0')
    elif ROS_ROBOT in ('Motoman','Motoman_SIM'):  ct.robot= TRobotMotoman()
    else:  ct.robot= TFakeRobot()

    if ct.robot.Is('PR2') or ct.robot.Is('Baxter') or ct.robot.Is('Motoman'):
      ct.state_validity_checker= TStateValidityCheckerMI()

    res= []
    ra= lambda r: res.append(r)

    ra(ct.robot.Init())
    if ct.robot.Is('PR2') or ct.robot.Is('Baxter') or ct.robot.Is('Motoman'):
      ra(ct.state_validity_checker.Init(ct.robot))

    if False in res:
      CPrint(4, 'Failed to setup in _default. Run init again.')

    if ROS_ROBOT in ('PR2','Baxter','BaxterN','Motoman'):
      ct.SetAttr('environment', 'real')
    elif ROS_ROBOT in ('PR2_SIM','Baxter_SIM','Motoman_SIM'):
      ct.SetAttr('environment', 'sim')

    ct.Run('model_loader')
    return

def Run(ct,*args):
  print 'Loading _default...'
  ct.thread_manager.Add(name='_default', target=lambda th_info,ct=ct: Main(ct,th_info))

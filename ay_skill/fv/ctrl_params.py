#!/usr/bin/python
from core_tool import *
def Help():
  return '''Set parameters for controllers with FingerVision.
  Usage: Execute ct.Run('fv.ctrl_params') from other scripts.'''
def Run(ct,*args):
  #Parameters used in fv.hold, fv.pickup2a, fv.pickup2b:
  ct.SetAttr('fv_ctrl','hold_sensitivity_slip', 0.08)  #Sensitivity of slip detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','hold_sensitivity_oc',0.2)  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','hold_sensitivity_oo',0.5)  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','hold_sensitivity_oa',0.4)  #Sensitivity of object-area-change detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','pickup2a_area_drop_ratio', 0.3)  #If object area becomes smaller than this ratio, it's considered as dropped.
  ct.SetAttr('fv_ctrl','pickup2a_z_final', 0.07)  #Final height (offset from the beginning).
  ct.SetAttr('fv_ctrl','pickup2a_obj_area_filter_len', 5)  #Filter length for obj_area.

  #Parameters used in fv.openif:
  ct.SetAttr('fv_ctrl','openif_sensitivity_slip', 0.6)  #Sensitivity of slip detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','openif_sensitivity_oc',0.4)  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','openif_sensitivity_oo',4.0)  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','openif_sensitivity_oa',0.6)  #Sensitivity of object-area-change detection (smaller is more sensitive).
  ct.SetAttr('fv_ctrl','openif_sensitivity_force',0.9)  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
  ct.SetAttr('fv_ctrl','openif_nforce_threshold', 20)  #Threshold of number of force changing points to open the gripper.
  ct.SetAttr('fv_ctrl','openif_dw_grip', 0.02)  #Displacement of gripper movement.

  #Designed for Baxter+Robotiq.
  ct.SetAttr('fv_ctrl','min_gstep', [0.0005,0.0005])
  ct.SetAttr('fv_ctrl','effort', [1.0,1.0])
  ct.SetAttr('fv_ctrl','tracko_gain', [[0.5,0.5,0.5]]*2)
  ct.SetAttr('fv_ctrl','trackf2_flen', 30)
  ct.SetAttr('fv_ctrl','trackf2_kp', 0.1)
  #ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 15.0,  1.0,1.0,1.0])
  #ct.SetAttr('fv_ctrl','pickup2a_kd', [0.1,0.1, 2.5,  0.1,0.1,0.1])
  ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 5.0,  1.0,1.0,1.0])
  ct.SetAttr('fv_ctrl','pickup2a_kd', [0.1,0.1, 1.0,  0.1,0.1,0.1])
  ct.SetAttr('fv_ctrl','pickup2a_lowgain', 0.3)
  ct.SetAttr('fv_ctrl','pickup2a_gtimeout1', 50)
  ct.SetAttr('fv_ctrl','pickup2a_gtimeout2', 250)
  for arm in range(ct.robot.NumArms):
    if ct.robot.EndEff(arm).Is('BaxterEPG'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.002
    elif ct.robot.EndEff(arm).Is('DxlGripper'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.002
      ct.GetAttr('fv_ctrl','effort')[arm]= 100.0
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout1', 40)  #Ctrl step=50Hz(Mikata), 500Hz(Bx)
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout2', 50)
    elif ct.robot.EndEff(arm).Is('ThGripper'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.0005
      ct.GetAttr('fv_ctrl','effort')[arm]= 100.0
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout1', 40)  #Ctrl step=50Hz(Mikata), 500Hz(Bx)
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout2', 50)
    elif ct.robot.EndEff(arm).Is('EZGripper'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.003
      ct.GetAttr('fv_ctrl','effort')[arm]= 100.0
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout1', 40)  #Ctrl step=50Hz(Mikata), 500Hz(Bx)
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout2', 50)
    elif ct.robot.EndEff(arm).Is('DxlO3'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.002
      ct.GetAttr('fv_ctrl','effort')[arm]= 100.0
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout1', 40)  #Ctrl step=50Hz(Mikata), 500Hz(Bx)
      ct.SetAttr('fv_ctrl','pickup2a_gtimeout2', 50)
    elif ct.robot.EndEff(arm).Is('DxlpY1Gripper'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.005
  if ct.robot.Is('Mikata'):
    for arm in range(ct.robot.NumArms):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.002
    ct.SetAttr('fv_ctrl','trackf2_flen', 10)
    ct.SetAttr('fv_ctrl','trackf2_kp', [0,0.3,0.3, 0.1,0.1,0.1])
    ct.SetAttr('fv_ctrl','tracko_gain', [[0.1,0.1,0.1]])
    ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 10.0,  1.0,1.0,1.0])
    ct.SetAttr('fv_ctrl','pickup2a_kd', [0.1,0.1, 1.0,  0.1,0.1,0.1])
    ct.SetAttr('fv_ctrl','pickup2a_lowgain', 0.1)
    ct.SetAttr('fv_ctrl','pickup2a_z_final', 0.05)
  elif ct.robot.Is('UR'):
    ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 4.0,  1.0,1.0,1.0])
    ct.SetAttr('fv_ctrl','pickup2a_kd', [0.01,0.01, 0.03,  0.01,0.01,0.01])
  elif ct.robot.Is('Gen3'):
    ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 4.0,  1.0,1.0,1.0])
    ct.SetAttr('fv_ctrl','pickup2a_kd', [0.01,0.01, 0.03,  0.01,0.01,0.01])

  #Load fv_ctrl parameters from files.
  CONFIG_FILE= 'fv_ctrl.yaml'
  if not ct.HasAttr('config_path'):
    ct.SetAttr('config_path', [os.path.join(os.environ['HOME'],subdir) for subdir in ['config/','data/config/',]])
  ctrl_params= {}
  for dir_path in ct.GetAttr('config_path'):
    file_path= os.path.join(dir_path,CONFIG_FILE)
    if os.path.exists(file_path):
      InsertDict(ctrl_params, LoadYAML(file_path))
  for k,v in ctrl_params.iteritems():
    ct.SetAttr('fv_ctrl',k, v)

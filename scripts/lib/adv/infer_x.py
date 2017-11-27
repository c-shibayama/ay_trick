#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer a pose of an object.
  Usage: adv.infer_x OBJ
    OBJ: ID of object.  e.g. 'b1'
  '''
def Run(ct,*args):
  obj= args[0]

  #Check the inference candidates and priority:
  method= ''
  #Infer x when obj is grabbed:
  if ct.HasAttr(obj,'grabbed'):
    method= 'grab'
  #Infer x when obj is on a kinematics chain:
  elif ct.HasAttr(obj,'kinematics_chain'):
    method= 'fk'
  else:
    score_ref= None
    score_base= None
    #Infer x when reference marker is known:
    if ct.HasAttr(obj,'ref_marker_id') and ct.HasAttr(obj,'ref_marker_pose'):
      if ct.GetAttr(obj,'ref_marker_id') in ct.ar_markers and len(ct.GetAttr(obj,'ref_marker_pose'))==7:
        score_ref= ct.ARXtime(ct.GetAttr(obj,'ref_marker_id'))
    #Infer x when base marker is known:
    if ct.HasAttr(obj,'base_marker_id'):
      if ct.GetAttr(obj,'base_marker_id') in ct.ar_markers:
        score_base= ct.ARXtime(ct.GetAttr(obj,'base_marker_id'))
    if score_ref!=None and score_base!=None:  #When both are observed, use new one
      if score_ref>=score_base:  method= 'refm'
      else:  method= 'basem'
    elif score_ref!=None:  method= 'refm'
    elif score_base!=None:  method= 'basem'
    #print 'score_ref:',score_ref
    #print 'score_base:',score_base

  #Infer x when obj is grabbed:
  if method=='grab':
    grabber_handid= ct.GetAttrOr(None,obj,'grabbed','grabber_handid')
    grabber_wrist= ct.GetAttrOr(None,obj,'grabbed','grabber_wrist')
    lo_x_grab= ct.GetAttrOr(None,obj,'l_x_grab')

    if None not in (grabber_handid,grabber_wrist,lo_x_grab):
      x_e= ct.robot.FK(x_ext=ct.GetAttr(grabber_wrist,'lx'),arm=grabber_handid)
      ct.SetAttr(obj,'x',  TransformRightInv(x_e, lo_x_grab))
    else:
      method= ''
      if verbose:
        print 'Some of following are missing:'
        print '  grabber_handid=',grabber_handid
        print '  grabber_wrist=',grabber_wrist
        print '  l_x_grab=',lo_x_grab

  #Infer x when obj is on a kinematics chain:
  elif method=='fk':
    parent_arm_id=  StrToLR(ct.GetAttrOr(None,obj,'kinematics_chain','parent_arm'))
    lx= ct.GetAttrOr(None,obj,'lx')
    if parent_arm_id is not None and lx is not None:
      ct.SetAttr(obj,'x',  ct.robot.FK(x_ext=lx, arm=parent_arm_id))
    else:
      method= ''
      if verbose:
        print 'Some of following are missing:'
        print '  [kinematics_chain][parent_arm]=',parent_arm_id
        print '  [lx]=',lx

  #Infer x when reference marker is known:
  elif method=='refm':
    ref_marker_id= ct.GetAttrOr(None,obj,'ref_marker_id')
    ref_marker_pose= ct.GetAttrOr(None,obj,'ref_marker_pose')
    if None not in (ref_marker_id,ref_marker_pose):
      if verbose:  print '###infer from ref',ref_marker_id
      x_m_ref= ct.ARX(ref_marker_id)
      ct.SetAttr(obj,'x',  TransformRightInv(x_m_ref, ref_marker_pose))
    else:
      method= ''
      if verbose:
        print 'Some of following are missing:'
        print '  ref_marker_id=',ref_marker_id
        print '  ref_marker_pose=',ref_marker_pose

  #Infer x when base marker is known:
  elif method=='basem':
    base_marker_id= ct.GetAttrOr(None,obj,'base_marker_id')
    if None is not base_marker_id:
      if verbose:  print '###infer from base',base_marker_id
      ct.SetAttr(obj,'x',  ct.ARX(base_marker_id))
    else:
      method= ''
      if verbose:
        print 'Some of following are missing:'
        print '  base_marker_id=',base_marker_id

  #If 'x' is inferred in some way, show the result and exit:
  if method!='':
    ExitProc(inferred=True,store=False)
    return True

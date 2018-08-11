#!/usr/bin/python
from core_tool import *
def Help():
  return '''Visualize a specified object.
  Usage: viz OBJ_ID1, OBJ_ID2, ...
    Start visualization.
    The scene objects (attribute[TMP]['scene']) are automatically visualized now.
    For only the current scene, run: viz ''
    OBJ_ID*: identifier of additional object. e.g. 'b1'
  Usage: viz
    Stop visualization. '''
def VizLoop(th_info, ct, objs):
  objs= set(objs)
  objs= objs-set(('',))
  objs= objs.union(ct.GetAttrOr([],TMP,'scene'))
  #tmpfp= file('/tmp/m_viz_state','w')
  viz= TSimpleVisualizer(rospy.Duration(1.0), name_space='visualizer_viz', frame=ct.GetAttr('default_frame'))
  m_infer= ct.Load('adv.infer_x')
  try:
    while th_info.IsRunning() and not rospy.is_shutdown():
      oid= 0
      #tmpfp.write('########Time: %r\n' % rospy.Time.now())
      #print 'p0',ct.robot.sensor_locker._is_owned(),ct.robot.sensor_locker._RLock__count
      if ct.robot.sensor_locker._RLock__count==0:  #FIXME: accessing row-level info. is this correct?
        xw=[ct.robot.FK(arm=arm) for arm in range(ct.robot.NumArms)]
      else:
        xw=[None for arm in range(ct.robot.NumArms)]
      mid= 0
      for obj in objs:
        #tmpfp.write('obj: %r\n' % obj)
        #oid= id(ct.GetAttr(obj))%(2**31)
        oid+= 100
        mid= oid
        #tmpfp.write('oid: %r\n' % oid)
        #print 'p0',ct.robot.sensor_locker._is_owned(),ct.robot.sensor_locker._RLock__count
        if m_infer is not None and ct.robot.sensor_locker._RLock__count==0:  #FIXME: accessing row-level info. is this correct?
          m_infer.Run(ct, obj)
          if ct.HasAttr(obj+'-cvste'):  #The mouth is tracked by cvstedge1
            m_infer.Run(ct, obj+'-cvste')
        x_o= ct.GetAttrOr(None, obj,'x')
        x_o_cvste= ct.GetAttrOr(None, obj+'-cvste','x')
        if x_o is None:
          #tmpfp.write('Failed to infer x_o\n')
          pass
        else:
          mid= viz.AddCoord(x_o, scale=[0.03,0.002], alpha=1.0, mid=mid)
          #tmpfp.write('x_o: %r\n' % x_o)
          primitives= ct.GetAttrOr(None, obj,'grab_primitives')
          if primitives is not None:
            alpha= 0.5
            for prm in primitives:
              if prm['kind']=='pkCylinder':
                p_1= Transform(x_o,prm['p1'])
                p_2= Transform(x_o,prm['p2'])
                mid= viz.AddCylinder(p_1,p_2, prm['width'], alpha=alpha, rgb=viz.ICol(5), mid=mid)
              elif prm['kind']=='pkCube':
                x_center= Transform(x_o,prm['x_center'])
                mid= viz.AddCube(x_center, scale=prm['dims'], alpha=alpha, rgb=viz.ICol(5), mid=mid)
          primitives= ct.GetAttrOr(None, obj,'shape_primitives')
          if primitives is not None:
            alpha= 0.6
            for prm in primitives:
              if prm['kind'] in ('rtpkCylinder','rtpkHalfCylinder'):
                #p_1= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,-0.5*prm['param'][1]])
                #p_2= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,+0.5*prm['param'][1]])
                p_1= Transform(x_o,Transform(prm['pose'],[0.,0.,-0.5*prm['param'][1]]))
                p_2= Transform(x_o,Transform(prm['pose'],[0.,0.,+0.5*prm['param'][1]]))
                mid= viz.AddCylinder(p_1,p_2, 2.0*prm['param'][0], alpha=alpha, rgb=viz.ICol(oid), mid=mid)
              elif prm['kind'] in ('rtpkTube','rtpkHalfTube'):
                #p_1= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,-0.5*prm['param'][2]])
                #p_2= Transform(x_o,Vec(prm['pose'][:3])+[0.,0.,+0.5*prm['param'][2]])
                p_1= Transform(x_o,Transform(prm['pose'],[0.,0.,-0.5*prm['param'][2]]))
                p_2= Transform(x_o,Transform(prm['pose'],[0.,0.,+0.5*prm['param'][2]]))
                mid= viz.AddCylinder(p_1,p_2, 2.0*prm['param'][0], alpha=alpha, rgb=viz.ICol(oid), mid=mid)
              elif prm['kind']=='rtpkCuboid':
                x_center= Transform(x_o,prm['pose'])
                mid= viz.AddCube(x_center, scale=2.0*Vec(prm['param']), alpha=alpha, rgb=viz.ICol(oid), mid=mid)
              elif prm['kind']=='rtpkRectTube':
                x_center= Transform(x_o,prm['pose'])
                mid= viz.AddCube(x_center, scale=2.0*Vec(prm['param'][:3]), alpha=alpha, rgb=viz.ICol(oid), mid=mid)
          l_x_pour_e= ct.GetAttrOr(None, obj,'l_x_pour_e')
          if l_x_pour_e is not None:
            x_pour_e= Transform(x_o,l_x_pour_e)
            mid= viz.AddSphere(x_pour_e, scale=[0.008]*3, rgb=viz.ICol(1), alpha=0.7, mid=mid)
            mid= viz.AddCoord(x_pour_e, scale=[0.05,0.002], alpha=1.0, mid=mid)
          l_x_pour_l= ct.GetAttrOr(None, obj,'l_x_pour_l')
          if l_x_pour_l is not None:
            x_pour_l= Transform(x_o,l_x_pour_l)
            mid= viz.AddSphere(x_pour_l, scale=[0.008]*3, rgb=viz.ICol(2), alpha=0.7, mid=mid)
            mid= viz.AddCoord(x_pour_l, scale=[0.05,0.002], alpha=1.0, mid=mid)
          l_p_pour_e_set= ct.GetAttrOr(None, obj,'l_p_pour_e_set')
          if l_p_pour_e_set is not None:
            p_pour_e_set= map(lambda p: Transform(x_o,p), l_p_pour_e_set)
            #print p_pour_e_set
            mid= viz.AddPoints(p_pour_e_set, scale=[0.008,0.008], rgb=viz.ICol(0), alpha=1.0, mid=mid)
            mid= viz.AddPolygon(p_pour_e_set, scale=[0.004], rgb=[1,0.5,0.5], alpha=1.0, mid=mid)
            if x_o_cvste is not None:  #The mouth is tracked by cvstedge1
              p_pour_e_set2= map(lambda p: Transform(x_o_cvste,p), l_p_pour_e_set)
              mid= viz.AddPoints(p_pour_e_set2, scale=[0.008,0.008], rgb=viz.ICol(1), alpha=1.0, mid=mid)
          l_x_grab= ct.GetAttrOr(None, obj,'l_x_grab')
          if l_x_grab is not None:
            x_g= Transform(x_o,l_x_grab)
            mid= viz.AddCube(x_g, scale=[0.06,0.04,0.01], rgb=viz.ICol(5), alpha=0.7, mid=mid)
            mid= viz.AddArrow(x_g, scale=[0.06,0.003,0.003], rgb=viz.ICol(5), alpha=0.7, mid=mid)
          #End of: if x_o is None else
        #tmpfp.write('mid: %r\n' % mid)
        #End of: for obj in objs
      if ct.HasAttr(TMP):
        xe_near_rcv= ct.GetAttrOr(None, TMP,'xe_near_rcv')
        if xe_near_rcv is not None:
          mid= viz.AddCube(xe_near_rcv, scale=[0.06,0.04,0.01], rgb=viz.ICol(7), alpha=0.7, mid=mid)
          mid= viz.AddArrow(xe_near_rcv, scale=[0.06,0.003,0.003], rgb=viz.ICol(7), alpha=0.7, mid=mid)
      #Visualize gripper models:
      for arm in range(ct.robot.NumArms):
        if xw[arm] is None:  continue
        if not ct.HasAttr('wrist_'+LRToStrs(arm),'lx'):  continue
        mid= viz.AddCoord(xw[arm], scale=[0.03,0.002], alpha=1.0, mid=mid)
        lw_xe= ct.GetAttr('wrist_'+LRToStrs(arm),'lx')
        bb_dim= ct.GetAttr('wrist_'+LRToStrs(arm),'bound_box','dim')
        bb_center= ct.GetAttr('wrist_'+LRToStrs(arm),'bound_box','center')
        mid= viz.AddCube(Vec(Transform(xw[arm],bb_center)), bb_dim, rgb=viz.ICol(3), alpha=0.5, mid=mid)
        #Visualize finger pads:
        gpos= ct.robot.GripperPos(arm)
        lw_xgl= Transform(lw_xe,[0,+0.5*gpos,0, 0,0,0,1])
        lw_xgr= Transform(lw_xe,[0,-0.5*gpos,0, 0,0,0,1])
        mid= viz.AddCube(Transform(xw[arm],lw_xgl), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
        mid= viz.AddCube(Transform(xw[arm],lw_xgr), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
        mid= viz.AddCoord(Transform(xw[arm],lw_xe), scale=[0.01,0.001], alpha=1.0, mid=mid)
      #Sentis M100 on Gripper:
      if ct.robot.Is('Baxter'):
        if xw[LEFT] is not None:
          lx_sensor_lg= ct.GetAttr('wl_m100','lx')
          #x_sensor= ct.robot.FK(x_ext=lx_sensor_lg, arm=LEFT)
          x_sensor= Transform(xw[LEFT],lx_sensor_lg)
          mid= viz.AddMarker(x_sensor, scale=[0.06,0.06,0.012], alpha=0.8, mid=mid)
          mid= viz.AddCoord(x_sensor, scale=[0.07,0.002], alpha=1.0, mid=mid)
      #Stereo camera on Gripper:
      if ct.robot.Is('Baxter'):
        if xw[RIGHT] is not None:
          lx_sensor_lg= ct.GetAttr('wr_stereo','lx')
          #x_sensor= ct.robot.FK(x_ext=lx_sensor_lg, arm=LEFT)
          x_sensor= Transform(xw[RIGHT],lx_sensor_lg)
          mid= viz.AddMarker(x_sensor, scale=[0.12,0.06,0.012], alpha=0.8, mid=mid)
          mid= viz.AddCoord(x_sensor, scale=[0.07,0.002], alpha=1.0, mid=mid)

      '''
      #Visualize the marker on the wrist
      if len(ct.l_x_m_wrist)==7 and xw[ct.ar_adjust_arm] is not None:
        #x_m_wrist= ct.robot.FK(x_ext=ct.l_x_m_wrist, arm=ct.ar_adjust_arm)
        x_m_wrist= Transform(xw[ct.ar_adjust_arm],ct.l_x_m_wrist)
        mid= viz.AddMarker(x_m_wrist, scale=[0.04,0.04,0.012], rgb=viz.ICol(1), alpha=0.5, mid=mid)
        mid= viz.AddArrow(x_m_wrist, scale=[0.05,0.004,0.004], rgb=viz.ICol(0), alpha=0.5, mid=mid)
        mid= viz.AddCoord(x_m_wrist, scale=[0.04,0.002], alpha=0.5, mid=mid)
      '''

      #Sleep until next visualization frame...
      time.sleep(100.0e-3)
      #End of: while Running...
  except Exception as e:
    PrintException(e, ' in viz')
  #tmpfp.write('########Finished: %r\n' % rospy.Time.now())
  #tmpfp.close()
def Run(ct,*args):
  if len(args)>0:
    ct.thread_manager.Add(name='viz', target=lambda th_info: VizLoop(th_info, ct,args))
  else:
    ct.thread_manager.Stop(name='viz')

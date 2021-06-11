#!/usr/bin/python
from core_tool import *
roslib.load_manifest('sensor_msgs')
import cv2
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError

def Help():
  return '''RealSense2 camera utility installed on the wrist of the robot arm.
  Usage:
    wrist_rs2 'on' [, OPTIONS]
    wrist_rs2 'setup' [, OPTIONS]
      Start to subscribe the topics.
      The observed data can be accessed via: ct.GetAttr(TMP,RS_ATTR)
      OPTIONS: Dictionary of options (default:DefaultOptions()).
      RS_ATTR: OPTIONS['rs_attr']
    wrist_rs2
    wrist_rs2 'off' [, RS_ATTR]
    wrist_rs2 'clear' [, RS_ATTR]
      Stop to subscribe topics.
    wrist_rs2 'show' [, RS_ATTR]
      Display the received images.
      RS_ATTR: Attribute name to access the data; default:'rs' (ct.GetAttr(TMP,RS_ATTR)).
  '''

def DefaultOptions():
  return {
    'types': ['depth'],  #List of subscribing components ('depth','rgb')
    'arm': 0,  #Arm index of the robot on whose wrist the camera is installed.
    'lx': [-0.035, 0.06, 0.0358, 0, 0, 0, 1],  #Pose of the camera (camera_color_optical_frame) in the wrist frame.
    'rs_attr': 'rs',  #Captured data is saved into: ct.GetAttr(TMP,rs_attr)
    }

def GetCameraProjectionMatrix(cam_info_topic='/camera/aligned_depth_to_color/camera_info'):
  try:
    cam_info= rospy.wait_for_message(cam_info_topic, sensor_msgs.msg.CameraInfo, 5.0)
    proj_mat= np.array(cam_info.P).reshape(3,4) #get camera projection matrix from ros topic
    return proj_mat
  except (rospy.ROSException, rospy.ROSInterruptException):
    raise Exception('Failed to read topic: {cam_info_topic}'.format(cam_info_topic=cam_info_topic))

def ReceiveDepth(ct,l,lh,msg):
  with lh.thread_locker:
    l.msg_depth= msg
    l.stamp_depth= msg.header
  try:
    with lh.thread_locker:
      #convert ros image to cv image (matrix) 16bit monochrome
      l.img_depth= lh.cvbridge.imgmsg_to_cv2(msg, '16UC1')
      if len(l.img_depth.shape)==3:  l.img_depth= l.img_depth.reshape(l.img_depth.shape[0],-1)  #NOTE: For the compatibility of cvbridge version difference(?)
  except CvBridgeError as e:
    print e
  if ct.robot is not None:
    with lh.thread_locker:
      l.xw= ct.robot.FK(arm=l.arm)
    ct.br.sendTransform(l.lw_x_camera_link[0:3],l.lw_x_camera_link[3:],
        rospy.Time.now(), 'camera_link', l.frame)
  if ct.callback.rs is not None:
    ct.callback.rs('depth',l,lh)

def ReceiveRGB(ct,l,lh,msg):
  with lh.thread_locker:
    l.msg_rgb= msg
    l.stamp_rgb= msg.header
  try:
    with lh.thread_locker:
      l.img_rgb= lh.cvbridge.imgmsg_to_cv2(msg, 'bgr8')
  except CvBridgeError as e:
    print e
  if ct.robot is not None and 'depth' not in l.options['types']:
    with lh.thread_locker:
      l.xw= ct.robot.FK(arm=l.arm)
    ct.br.sendTransform(l.lw_x_camera_link[0:3],l.lw_x_camera_link[3:],
        rospy.Time.now(), 'camera_link', l.frame)
  if ct.callback.rs is not None:
    ct.callback.rs('rgb',l,lh)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  def UnSubscribe(rs_attr):
    ct.DelSub('{rs_attr}_depth'.format(rs_attr=rs_attr))
    ct.DelSub('{rs_attr}_rgb'.format(rs_attr=rs_attr))

  if command in ('on','setup'):
    user_options= args[0] if len(args)>0 else {}
    options= DefaultOptions()
    InsertDict(options, user_options)
    UnSubscribe(options['rs_attr'])

    ct.SetAttr(TMP,'{rs_attr}_helper'.format(rs_attr=options['rs_attr']), TContainer())
    lh= ct.GetAttr(TMP,'{rs_attr}_helper'.format(rs_attr=options['rs_attr']))
    lh.cvbridge= CvBridge()
    lh.thread_locker= threading.RLock()
    ct.SetAttr(TMP,options['rs_attr'], TContainer())
    l= ct.GetAttr(TMP,options['rs_attr'])
    l.options= options
    l.proj_mat= GetCameraProjectionMatrix()
    l.msg_depth= None  #Original depth message.
    l.img_depth= None  #Depth image (for OpenCV).
    l.stamp_depth= None  #Stamp of the depth message.
    l.msg_rgb= None  #Original rgb message.
    l.img_rgb= None  #RGB image (for OpenCV).
    l.stamp_rgb= None  #Stamp of the rgb message.
    ct.callback.rs= None
    #Arm on which the RealSense is attached.
    l.arm= options['arm']
    #Pose of RealSense (camera_color_optical_frame) in the wrist frame.
    l.frame= ct.robot.EndLink(l.arm)
    l.lx= options['lx']
    l.lw_x_camera_link= TransformRightInv(l.lx,ct.Run('tf_once','camera_link','camera_color_optical_frame'))
    #Wrist pose at the observation. If both depth and rgb are observed, xw is measured only when depth is observed.
    l.xw= None

    if 'depth' in options['types']:
      ct.AddSubW('{rs_attr}_depth'.format(rs_attr=options['rs_attr']),
                 '/camera/aligned_depth_to_color/image_raw', sensor_msgs.msg.Image, lambda msg,ct=ct,l=l,lh=lh:ReceiveDepth(ct,l,lh,msg), time_out=3.0)
    if 'rgb' in options['types']:
      ct.AddSubW('{rs_attr}_rgb'.format(rs_attr=options['rs_attr']),
                 '/camera/color/image_raw', sensor_msgs.msg.Image, lambda msg,ct=ct,l=l,lh=lh:ReceiveRGB(ct,l,lh,msg), time_out=3.0)

  elif command in ('off','clear'):
    rs_attr= args[0] if len(args)>0 else 'rs'
    UnSubscribe(rs_attr)

  elif command=='show':
    rs_attr= args[0] if len(args)>0 else 'rs'
    components= ct.GetAttr(TMP,rs_attr).options['types']
    print 'Press any key to quit.'
    kbhit= TKBHit()
    try:
      for c in components:
        cv2.namedWindow(c)
      while True:
        if kbhit.IsActive():
          key= kbhit.KBHit()
          if key is not None:
            break
        else:
          break
        if 'depth' in components:  cv2.imshow('depth',ct.GetAttr(TMP,rs_attr).img_depth*255)
        if 'rgb' in components:  cv2.imshow('rgb',ct.GetAttr(TMP,rs_attr).img_rgb)
        key= cv2.waitKey(10)&0xFF
        if key==ord('q'):
          break
    finally:
      kbhit.Deactivate()
      cv2.destroyAllWindows()
      for _ in range(10):  cv2.waitKey(10)


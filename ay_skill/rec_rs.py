#!/usr/bin/python
from core_tool import *
import six.moves.cPickle as pickle
import gzip
import cv2
def Help():
  return '''Record RealSense observations into gzip-ed pickle files.
  Note: RealSense should be activated by one of follows:
    > robot_rs2 'on'
    > wrist_rs2 'on'
  Usage: rec_rs [DATA_DIR [, RS_ATTR [, PREFIX [, SUFFIX]]]]
    DATA_DIR: Directory to store the data files (default: ct.DataBaseDir()+'tmp').
    RS_ATTR: RealSense attribute (default: 'rs').
    PREFIX,SUFFIX: Prefix and suffix of a data file (default:'rs','.dat').
  NOTE: We do not use Python gzip module due to its slowness, but use the system gzip command.
  NOTE: The depth and RGB images are also saved in similar filenames for the convenience (they are included in the pickled data).
  NOTE: In order to read the depth image (1ch, uint16) from OpenCV, use imread like:
    > cv2.imread('rsxxx-depth.png', cv2.IMREAD_ANYDEPTH)'''

def Run(ct,*args):
  data_dir= args[0] if len(args)>0 else ct.DataBaseDir()+'tmp'
  rs_attr= args[1] if len(args)>1 else 'rs'
  prefix= args[2] if len(args)>2 else 'rs'
  suffix= args[3] if len(args)>3 else '.dat'

  try:
    with TKBHit() as kbhit:
      print '''q: Quit.\nSpace/r: Record.'''
      for c in ('rgb','depth'):  cv2.namedWindow(c)
      rec_req= False
      while not rospy.is_shutdown():
        if kbhit.IsActive():
          key= kbhit.KBHit()
          if key=='q':  break
          elif key in (' ','r'):  rec_req= True
        else:  break

        if ct.GetAttr(TMP,rs_attr).img_depth is not None:
          cv2.imshow('depth',ct.GetAttr(TMP,rs_attr).img_depth*255)
        if ct.GetAttr(TMP,rs_attr).img_rgb is not None:
          cv2.imshow('rgb',ct.GetAttr(TMP,rs_attr).img_rgb)
        key= cv2.waitKey(10)&0xFF
        if key==ord('q'):  break
        elif key in (ord(' '),ord('r')):  rec_req= True

        if rec_req:
          rec_req= False
          t_start= rospy.Time.now()
          with ct.GetAttr(TMP,'rs_helper').thread_locker:
            rs= copy.deepcopy(ct.GetAttr(TMP,rs_attr))
          timestamp= TimeStr('short2')
          #Save images:
          depth_file_name= '{}/{}{}-depth{}'.format(data_dir, prefix, timestamp, '.png')
          rgb_file_name=   '{}/{}{}-rgb{}'.format(data_dir, prefix, timestamp, '.png')
          cv2.imwrite(depth_file_name, rs.img_depth)
          cv2.imwrite(rgb_file_name, rs.img_rgb)
          CPrint(3,'Saved:',depth_file_name)
          CPrint(3,'Saved:',rgb_file_name)
          #Reduce the data size of rs object (TContainer):
          rs.msg_depth_header= rs.msg_depth.header
          rs.msg_rgb_header= rs.msg_rgb.header
          rs.msg_depth= None
          rs.msg_rgb= None
          #Save rs object (TContainer):
          log_file_name= '{}/{}{}{}'.format(data_dir, prefix, timestamp, suffix)
          with open(log_file_name, 'w') as log_fp:
            pickle.dump(rs,log_fp)
          os.system('gzip {0}'.format(log_file_name))
          CPrint(3,'Saved:',log_file_name+'.gz','({:.2f}s)'.format((rospy.Time.now()-t_start).to_sec()))
  finally:
    cv2.destroyAllWindows()
    for _ in range(10):  cv2.waitKey(10)

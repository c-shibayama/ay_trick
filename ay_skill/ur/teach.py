#!/usr/bin/python
from core_tool import *
import std_msgs.msg
def Help():
  return '''On/off the teaching mode of UR by hitting the space key.
  Usage: ur.teach'''
def SendReq(ct,msg):
  def loop(th_info):
    #for _ in range(250):
    while True:
      if not th_info.IsRunning():  break
      ct.pub.urscript.publish(msg)
      rospy.sleep(0.008)
  ct.thread_manager.Stop(name='ur_teach_loop')
  if msg!='':
    ct.thread_manager.Add(name='ur_teach_loop',target=loop)
def Run(ct,*args):
  try:
    ct.AddPub('urscript','/ur_driver/URScript',std_msgs.msg.String)
    state= 'not_teach'
    print 'Current mode:',state
    print 'Hit the space key to change the mode.'
    kbhit= TKBHit()
    while not rospy.is_shutdown():
      c= kbhit.KBHit()
      if c=='q':  break
      elif c==' ':
        state= 'not_teach' if state=='teach' else 'teach'
        print 'Current mode:',state
        if state=='teach':
          SendReq(ct,'teach_mode()\n')
        elif state=='not_teach':
          SendReq(ct,'end_teach_mode()\n')
      rospy.sleep(0.01)
  finally:
    kbhit.Deactivate()
    SendReq(ct,'')
    ct.pub.urscript.publish('end_teach_mode()\n')
    ct.DelPub('urscript')

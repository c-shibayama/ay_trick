#!/usr/bin/python
from core_tool import *
def Help():
  return '''Sentis M100 controller.
  Usage:
    m100
    m100 'clear'
      Stop all background processes.
    m100 'setup'
      Start sending tf broadcaster.
    m100 'rate' FPS
      Set frame rate to FPS.
      FPS: Frames per second.
    m100 'slow'
      Slowdown the frame rate (2 Hz).
  '''

def TfBroadcast(th_info, ct):
  lx_sensor_lg= ct.GetAttr('wl_m100','lx')
  rtposeest_x_sensor= lambda: ct.robot.FK(x_ext=lx_sensor_lg, arm=LEFT)
  rate= ct.GetAttrOr(2, 'wl_m100','rate')
  rate_adjuster= rospy.Rate(rate)

  while th_info.IsRunning() and not rospy.is_shutdown():
    x_sensor= rtposeest_x_sensor()
    ct.br.sendTransform(x_sensor[0:3],x_sensor[3:],
        rospy.Time.now(),
        'tf_sentis_tof',
        ct.robot.BaseFrame)
    rate_adjuster.sleep()

def Run(ct,*args):
  if len(args)>0:
    command= args[0]
    args= args[1:]
  else:
    command= 'clear'
    args= []

  def RunThread():
    ct.thread_manager.Add(name='m100_br', target=lambda th_info: TfBroadcast(th_info, ct))

  def SetRate(rate):
    ct.AddSrvP('m100_set_frame_rate', '/sentis_m100/set_frame_rate', lfd_vision.srv.SetFrameRate, persistent=False, time_out=3.0)
    ct.srvp.m100_set_frame_rate(rate)
    ct.SetAttr('wl_m100','rate', rate)
    if ct.thread_manager.IsRunning('m100_br'):
      RunThread()  #Relaunch the thread to renew the rate.

  if command=='setup':
    RunThread()

  elif command=='clear':
    ct.thread_manager.Stop(name='m100_br')

  elif command=='rate':
    SetRate(args[0])

  elif command=='slow':
    SetRate(2)

#!/usr/bin/python
from core_tool import *
import matplotlib.pyplot as plt

def Help():
  return '''Utility to plot data with matplotlib.
    This is currently an alpha version.
  Usage:
  > plot x,y
    If both x,y are arrays: Plot x, y.
    If x is an array and y is a function: Plot x, y(x).
    If x is a function and y is an array: Realtime plot x(), y.
    If both x,y are functions: Realtime plot x(), y().
  > plot 'reset_config'
    Remove and recreate the plot configuration in ct.GetAttr('plot').
  Plot:
    In default, a line and a scatter plots are made for x,y.
    The plot can be configured with the container stored in: ct.GetAttr('plot').
    If ct.GetAttr('plot') does not exist, it is automatically created.
  Realtime plot:
    Like an oscilloscope, the plot is made and updated continuously.
    For each cycle in the loop, x(), y() are computed and stored into lists X, Y.
    Then X, Y are plotted similar to the regular plot.
    The cycle interval is ct.GetAttr('plot').rt_interval.
    The sizes of X, Y are limited to ct.GetAttr('plot').rt_num_points.
  '''

def MakePlotConfig(ct):
  c= TContainer()
  c.mode='line_scatter'  #Plot mode, can be chosen from 'line', 'scatter', 'line_scatter'.
  c.title='plot'
  c.xlabel='x'
  c.ylabel='y'
  c.legend=True  #Whether run ax.legend().
  c.show=False  #Whether run plt.show().
  c.line_args=dict(color='blue')  #Options for plot.
  c.scatter_args=dict(color='red')  #Options for scatter.
  #c.rt_thread_name='plot'  #Thread name of realtime plot.
  c.rt_interval=0.01  #Sleep duration [sec] of realtime plot.
  c.rt_num_points=100  #Number of points to hold in realtime plot.
  ct.SetAttr('plot', c)

def Run(ct,*args,**kwargs):
  if args[0]=='make_config':
    MakePlotConfig(ct)
    return
  elif args[0]=='reset_config':
    ct.DelAttr('plot')
    MakePlotConfig(ct)
    return
  #elif args[0]=='stop':
    #if ct.HasAttr('plot'):
      #c= ct.GetAttr('plot')
      #print 'Stopping realtime plot thread:',c.rt_thread_name
      #ct.thread_manager.Stop(name=c.rt_thread_name)
    #return

  x= args[0]
  y= args[1]

  is_realtime_plot= False
  if not callable(x) and callable(y):
    y_func= y
    y= y_func(x)
  elif callable(x) and callable(y):
    is_realtime_plot= True
  elif callable(x) and not callable(y):
    is_realtime_plot= True
    y_data= y
    y= lambda x: y_data

  if not ct.HasAttr('plot'):  MakePlotConfig(ct)
  c= ct.GetAttr('plot')

  fig= plt.figure()
  ax= fig.add_subplot(1,1,1, title=c.title, xlabel=c.xlabel, ylabel=c.ylabel)

  if not is_realtime_plot:
    if 'line' in c.mode:  ax.plot(x, y, **c.line_args)
    if 'scatter' in c.mode:  ax.scatter(x, y, **c.scatter_args)
    if c.legend:  ax.legend()
    if c.show:  plt.show()

  else:
    #NOTE: We omitted to make a thread for plot as matplotlib is not thread-safe.
    #  To run this in background, we would need to implement a multiprocessing.
    #print 'Starting realtime plot thread:',c.rt_thread_name
    #ct.thread_manager.Stop(name=c.rt_thread_name)
    #ct.thread_manager.Add(name=c.rt_thread_name,
                          #target=lambda th_info,ct=ct,ax=ax,x=x,y=y: PlotLoop(th_info,ct,ax,x,y))
    X= []
    Y= []
    with TKBHit() as kbhit:
      try:
        while not rospy.is_shutdown():
          key= None
          if kbhit.IsActive():
            key= kbhit.KBHit()
            #if key=='q':  break
            if key is not None and len(key)>0:  break
          else:  break

          ax.cla()
          X.append(x())
          Y.append(y(X[-1]))
          while len(X)>c.rt_num_points:  X.pop(0)
          while len(Y)>c.rt_num_points:  Y.pop(0)
          if 'line' in c.mode:  ax.plot(X, Y, **c.line_args)
          if 'scatter' in c.mode:  ax.scatter(X, Y, **c.scatter_args)
          ax.set_title(c.title)
          ax.set_xlabel(c.xlabel)
          ax.set_ylabel(c.ylabel)
          if c.legend:  ax.legend()
          plt.pause(c.rt_interval)
          #plt.show()
          #rospy.sleep(c.rt_interval)
      except Exception as e:
        print 'Exception:',e


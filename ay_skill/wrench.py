#!/usr/bin/python
from core_tool import *
roslib.load_manifest('geometry_msgs')
import geometry_msgs.msg

def Help():
  return '''Force/torque sensor utility (e.g. a UR's wrist F/T sensor).
  Usage:
    wrench 'on' [, OPTIONS]
    wrench 'setup' [, OPTIONS]
      Start to subscribe the topics.
      The observed data can be accessed via: ct.GetAttr(TMP,FT_ATTR)
      OPTIONS: Dictionary of options (default:DefaultOptions()).
      FT_ATTR: OPTIONS['ft_attr']
    wrench
    wrench 'off' [, FT_ATTR]
    wrench 'clear' [, FT_ATTR]
      Stop to subscribe topics.
    wrench 'show' [, FT_ATTR]
      Display the received images.
      FT_ATTR: Attribute name to access the data; default:'ft' (ct.GetAttr(TMP,FT_ATTR)).
  '''

def DefaultOptions():
  return {
    'ft_attr': 'ft',  #Obtained data is saved into: ct.GetAttr(TMP,ft_attr)
    'topic': '/wrench',  #Wrench topic.
    }

def CallbackWrench(ct, l, msg):
  l.msg= msg
  if ct.callback.ft is not None:
    ct.callback.ft(l)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  def UnSubscribe(ft_attr):
    ct.DelSub('wrench_{ft_attr}'.format(ft_attr=ft_attr))

  if command in ('on','setup'):
    user_options= args[0] if len(args)>0 else {}
    options= DefaultOptions()
    InsertDict(options, user_options)
    UnSubscribe(options['ft_attr'])

    ct.SetAttr(TMP,options['ft_attr'], TContainer())
    l= ct.GetAttr(TMP,options['ft_attr'])
    l.options= options
    l.msg= None
    ct.callback.ft= None
    ct.AddSub('wrench_{ft_attr}'.format(ft_attr=options['ft_attr']),
              options['topic'], geometry_msgs.msg.WrenchStamped, lambda msg,ct=ct,l=l:CallbackWrench(ct,l,msg))

  elif command in ('off','clear'):
    ft_attr= args[0] if len(args)>0 else 'ft'
    UnSubscribe(ft_attr)

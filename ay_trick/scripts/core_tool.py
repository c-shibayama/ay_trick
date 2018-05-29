#! /usr/bin/env python
import roslib; roslib.load_manifest('ay_trick')
import pkgutil
from ay_py.core import *
from ay_py.ros import *

#Attribute key for temporary memory.
TMP='*'

def Import(modname):
  return __import__(modname,globals(),globals(),modname,-1)


class TCoreTool(TROSUtil):

  def __init__(self):
    super(TCoreTool,self).__init__()

    #self.library_path_prefix= 'lib.'
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))+'/../../ay_skill')
    sys.path.insert(0, os.environ['HOME']+'/ay_skill')
    self.loaded_libraries= []

    self.robot= None
    self.state_validity_checker= None

    '''
    Attribute is a kind of shared memory that is shared among library scripts.
    The member variable attributes is defined as a dictionary,
    which maps a key string to value.
    Usually use GetAttr, SetAttr, etc. to manipulate attributes since
    they provide safe accessing methods.
    '''
    #Attributes of objects
    self.attributes= {}
    #Thread locker for self.attributes:
    self.attr_locker= threading.RLock()

    '''
    Thread manager.
    It will be more convenient to use TCoreTool.thread_manager
    than making a thread in each library script,
    since we can stop all threads at once, investigate threads, etc..
    '''
    self.thread_manager= TThreadManager()

    '''
    Utility containers:
    Note: TContainer is a container class to which you can add any member variables.
    e.g. c= TContainer(); c.var1= 10; c.var2= 20;
    '''

    #Visualization list; element should be a TSimpleVisualizer
    self.viz= TContainer()
    #Store callback functions
    self.callback= TContainer()
    #For caching modules loaded by Load
    self.m= TContainer()

    '''
    Other utility variables:
    '''

    #tf broad caster.
    self.br= None


  def __del__(self):
    #self.stopRecord()
    self.Cleanup()
    super(TCoreTool,self).__del__()
    print 'TCoreTool: done',self

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency

    #Check the thread lockers status:
    print 'Count of attr_locker:',self.attr_locker._RLock__count

    self.thread_manager.StopAll()

    if self.state_validity_checker is not None:
      del self.state_validity_checker
      self.state_validity_checker= None
    if self.robot is not None:
      self.robot.Cleanup()
      del self.robot
      self.robot= None

    for k in self.callback.keys():
      print 'Stopping callback %r...' % k,
      self.callback[k]= None  #We do not delete
      print 'ok'

    for k in self.viz.keys():
      print 'Stop visualizing %r...' % k,
      del self.viz[k]
      print 'ok'

    super(TCoreTool,self).Cleanup()

    for k in self.m.keys():
      print 'Deleting library script cache %r...' % k,
      del self.m[k]
      print 'ok'

  @staticmethod
  def DataBaseDir():
    return '%s/data/' % (os.environ['HOME'])

  @staticmethod
  def LogFileName(prefix, timestamp=None, suffix='.dat'):
    if timestamp is None:  timestamp= TimeStr('short2')
    location= '{base}tmp'.format(base=TCoreTool.DataBaseDir())
    if not os.path.exists(location):
      CPrint(2,'Directory for log files does not exist:',location)
      CPrint(2,'Want to create? (if No, we will use /tmp)')
      if AskYesNo():  os.makedirs(location)
      else:  location= '/tmp'
    return '%s/%s%s%s' % (location, prefix, timestamp, suffix)



  def GetAttr(self,*keys):
    with self.attr_locker:
      d= self.attributes
      for n in keys:
        d= d[n]
      if isinstance(d,(int,bool,float)):  return d
      #return copy.deepcopy(d)  #TODO: if we have a 'lazydeepcopy', the performance will be improved
      return d  #TODO: if we have a 'lazydeepcopy', the performance will be improved
  def GetAttrOr(self,default,*keys):
    with self.attr_locker:
      d= self.attributes
      for n in keys:
        if type(d)!=dict or (not n in d):
          return default
        d= d[n]
      if isinstance(d,(int,bool,float)):  return d
      #return copy.deepcopy(d)  #TODO: if we have a 'lazydeepcopy', the performance will be improved
      return d  #TODO: if we have a 'lazydeepcopy', the performance will be improved
  def HasAttr(self,*keys):
    with self.attr_locker:
      d= self.attributes
      for n in keys:
        if type(d)!=dict or (not n in d):
          return False
        d= d[n]
      return True
  #Set a value to attributes; last element of keys_value is the value
  def SetAttr(self,*keys_value):
    assert(len(keys_value)>=2)
    with self.attr_locker:
      d= self.attributes
      for n in keys_value[0:-1]:
        if type(d)==dict and n in d:
          pass
        elif type(d)==dict:
          d[n]= {}
        else:  #i.e. type(d)!=dict (Note: at first, d==self.attributes should be a dict)
          last_d[last_n]= {}
          d= last_d[last_n]
          d[n]= {}
        last_d= d
        last_n= n
        d= d[n]
      last_d[last_n]= keys_value[-1]
  #Add a dictionary value to attributes (only the assignment behavior is different from SetAttr);
  #last element of keys_dvalue is the (dict) value
  def AddDictAttr(self,*keys_dvalue):
    dict_value= keys_dvalue[-1]
    assert(type(dict_value)==dict)
    assert(len(keys_dvalue)>=1)
    with self.attr_locker:
      d= self.attributes
      if len(keys_dvalue)==1:
        InsertDict(d, dict_value)
        return
      for n in keys_dvalue[0:-1]:
        if type(d)==dict and n in d:
          pass
        elif type(d)==dict:
          d[n]= {}
        else:  #i.e. type(d)!=dict (Note: at first, d==self.attributes should be a dict)
          last_d[last_n]= {}
          d= last_d[last_n]
          d[n]= {}
        last_d= d
        last_n= n
        d= d[n]
      InsertDict(last_d[last_n], dict_value)
  def DelAttr(self,*keys):
    with self.attr_locker:
      d= self.attributes
      if len(keys)==0:  return False
      for n in keys:
        if type(d)!=dict or (not n in d):
          return False
        last_d= d
        last_n= n
        d= d[n]
      del last_d[last_n]
      return True

  #Check if the module exists.
  def Exists(self, fileid):
    modname= fileid
    #return os.path.exists(modname.replace('.','/')+'.py')
    try:
      return (pkgutil.find_loader(modname) is not None)
    except ImportError:
      return False

  #Load external library script written in python,
  #which is imported as a module to this script, so we can share the memory
  def Load(self, fileid):
    modname= fileid
    try:
      mod= Import(modname)
      if modname in self.loaded_libraries:
        reload(mod)
      else:
        self.loaded_libraries.append(modname)
    except ImportError as e:
      PrintException(e)
      print 'Cannot import the library file: ',modname
      mod= None
    return mod

  #Execute external library script written in python,
  #which is imported as a module to this script, so we can share the memory
  def Run(self, fileid, *args):
    mod= self.Load(fileid)
    if mod:
      try:
        runfunc= mod.Run
      except AttributeError:
        runfunc= None
        print 'Run function is not defined in:', fileid
      if runfunc is not None:  return runfunc(self,*args)
      else:  return None


if __name__ == '__main__':
  print 'Use cui_tool.py or direct_run.py'




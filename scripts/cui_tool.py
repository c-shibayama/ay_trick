#! /usr/bin/env python
import roslib; roslib.load_manifest('ay_trick')
import rospy
from core_tool import *
#For CUI
import readline
import threading

#Instance of TCoreTool()
ct= None

def Eval(eval_line):
  return eval(eval_line,globals(),globals())

def Exec(exec_line):
  #return exec(exec_line,globals(),globals())
  eval_mode= True
  try:
    code_obj= compile(exec_line, '<string>', 'eval')
  except SyntaxError:
    code_obj= compile(exec_line, '<string>', 'exec')
    eval_mode= False
  if eval_mode:  return eval(code_obj,globals(),globals())
  exec(code_obj,globals(),globals())
  return None

class TCUITool(object):

  def __init__(self):
    global ct

    self.hist_file= '.trick_hist'
    try:
      readline.read_history_file(self.hist_file)
    except IOError:
      pass
    readline.parse_and_bind('tab: complete')
    self.write_history_file= readline.write_history_file

    ct= TCoreTool()

    self.thread_cui= None
    self.running= False
    self.done_exit_proc= False

  def __del__(self):
    global ct
    self.Exit('TCUITool.__del__')
    del ct
    ct= None

  def Exit(self,where='',wait_cui=True):
    global ct
    self.running= False
    if wait_cui and self.thread_cui is not None:  self.thread_cui.join()

    #exit_locker= threading.RLock()
    #with exit_locker:
    if not self.done_exit_proc:
      print 'Exiting at %r...' % where
      self.SaveHistory()
      ct.Run('_exit')
      print 'TCoreTool.Cleanup...'
      ct.Cleanup()
      rospy.signal_shutdown('quit...')
      self.done_exit_proc= True

  def SaveHistory(self):
    print 'Save history into ',self.hist_file
    self.write_history_file(self.hist_file)

  def Start(self):
    global ct
    ct.Run('_default')

    self.thread_cui= threading.Thread(name='thread_cui', target=self.Interface)
    self.thread_cui.start()

  def Interface(self):
    global ct
    self.running= True
    while self.running and not rospy.is_shutdown():
      if ct.robot is not None:  caret= '%s:trick or quit|%s> '%(ct.robot.Name,ct.robot.ArmStrS())
      else:  caret= 'trick or quit> '
      try:
        cmd_raw= raw_input(caret).strip()
      except EOFError:
        self.running= False
        continue
      cmd_split= cmd_raw.split()

      try:
        if cmd_raw=='':
          continue
        elif cmd_raw == 'quit' or cmd_raw == 'exit':
          self.running= False
          continue
        else:
          if ct.Exists(cmd_split[0]):
            if len(cmd_split)==2 and cmd_split[1] in ('-help','--help'):
              sub= ct.Load(cmd_split[0])
              try:
                helpfunc= sub.Help
              except AttributeError:
                helpfunc= None
                print 'Help function is not defined in:', cmd_split[0]
              if helpfunc is not None:  print helpfunc()
            else:
              if len(cmd_split)>1:  args= Eval(' '.join(cmd_split[1:]))
              else:  args= ()
              if not isinstance(args,tuple):  args= (args,)
              res= ct.Run(cmd_split[0], *args)
              if res!=None:  print res
          else:
            res= Exec(cmd_raw)
            if res!=None:  print res
      except Exception as e:
        PrintException(e,' in CUI')
        c1,c2,ce= ACol.I(4,1,None)
        print '%sCheck the command line: %s%s %s' % (c1, c2,cmd_raw, ce)

    self.Exit('the end of TCUITool.Interface',wait_cui=False)


if __name__ == '__main__':
  rospy.init_node('cui_tool')
  cui= TCUITool()
  cui.Start()
  rospy.spin()
  cui.Exit('__main__')


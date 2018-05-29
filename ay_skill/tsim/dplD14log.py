#!/usr/bin/python
if __name__!='__main__':
  from core_tool import *
def Help():
  return '''Dump log for plot resulting from tsim.dplD14.
  Usage: tsim.dplD14log'''

def Run(ct,*args):
  #return
  if len(args)>0:
    l= args[0]
    SaveYAML(l.dpl.MM.Save(l.dpl.MM.Options['base_dir']), l.dpl.MM.Options['base_dir']+'model_mngr.yaml')
    SaveYAML(l.dpl.DB.Save(), l.logdir+'database.yaml')
    SaveYAML(l.dpl.Save(), l.logdir+'dpl.yaml')
  else:
    l= TContainer(debug=True)
    l.dpl= ct.log_dpl
    l.logdir= '/tmp/dpl/'

  #'''
  #Analyze l.dpl.DB.Entry:
  ptree= l.dpl.GetPTree('n0', {})
  fp= open(l.logdir+'dpl_est.dat','w')
  for i,eps in enumerate(l.dpl.DB.Entry):
    n0_0= eps.Find(('n0',0))[0]
    if n0_0 is None or eps.R is None:
      CPrint(4, 'l.dpl.DB has a broken entry')
      continue
    ptree.StartNode.XS= n0_0.XS
    ptree.ResetFlags()
    values= [eps.R, l.dpl.Value(ptree)]
    fp.write('%i %s\n' % (i, ' '.join(map(str,values))))
  fp.close()
  CPrint(1,'Generated:',l.logdir+'dpl_est.dat')
  #'''

def PlotGen(logdir):
  def check_exist(file_name):
    return os.path.exists(logdir+file_name) and os.path.getmtime(logdir+'dpl_log.dat')<os.path.getmtime(logdir+file_name)
  ex_dpl_sdump_s= check_exist('dpl_sdump_s.dat')
  ex_dpl_sdump_x= check_exist('dpl_sdump_x.dat')
  ex_dpl_sdump_y= check_exist('dpl_sdump_y.dat')
  if all((ex_dpl_sdump_s,ex_dpl_sdump_x,ex_dpl_sdump_y)):
    return
  CPrint(1,'Loading database and generating serialized data...')
  db= TGraphEpisodeDB()
  db.Load(LoadYAML(logdir+'dpl_log.dat'))
  if not ex_dpl_sdump_s:
    db.SDump(open(logdir+'dpl_sdump_s.dat','w'),
            (None,-1,'a_pour','a_spill2'))
  if not ex_dpl_sdump_x:
    db.SDump(open(logdir+'dpl_sdump_x.dat','w'),
            ('n0',0,'skill','gh_ratio','p_pour_trg0','p_pour_trg','size_srcmouth','material2'),
            (None,-1,'a_pour','a_spill2'))
  if not ex_dpl_sdump_y:
    db.SDump2(open(logdir+'dpl_sdump_y.dat','w'),
            ('n2c',None,lambda n:[len(n)]),  #How many times n2c (skill selection node) was visited?
            ('n3ti',None,lambda n:[len(n)]),  #How many times Tipping was used?
            ('n3sa',None,lambda n:[len(n)]),  #How many times Shaking-A was used?
            ('n4tir',None,lambda nn:[sum(max(0.0,n.XS['da_pour'].X[0,0]-n.XS['da_trg'].X[0,0]) for n in nn)]),  #Sum of max(0,da_pour-da_trg) of Tipping.
            ('n4tir',None,lambda nn:[sum(min(0.0,n.XS['da_pour'].X[0,0]-n.XS['da_trg'].X[0,0]) for n in nn)]),  #Sum of min(0,da_pour-da_trg) of Tipping.
            ('n4tir',None,lambda nn:[sum(n.XS['da_spill2'].X[0,0] for n in nn)]),  #Sum of da_spill2 of Tipping.
            ('n4sar',None,lambda nn:[sum(max(0.0,n.XS['da_pour'].X[0,0]-n.XS['da_trg'].X[0,0]) for n in nn)]),  #Sum of max(0,da_pour-da_trg) of Shaking-A.
            ('n4sar',None,lambda nn:[sum(min(0.0,n.XS['da_pour'].X[0,0]-n.XS['da_trg'].X[0,0]) for n in nn)]),  #Sum of min(0,da_pour-da_trg) of Shaking-A.
            ('n4sar',None,lambda nn:[sum(n.XS['da_spill2'].X[0,0] for n in nn)]) ) #Sum of da_spill2 of Shaking-A.

if __name__=='__main__':
  import os,sys
  logdir= sys.argv[1] if len(sys.argv)>1 else '/tmp/dpl/'
  qopt= sys.argv[2] if len(sys.argv)>2 else ''
  PlotGen(logdir)
  commands=[
    '''qplot -x2 go
          -s 'set title "Sum of rewards, Poured, Spilled"'
          -s 'set ytic nomirror;set y2tic'
          -s 'set xlabel "Num of episode"'
          -s 'set ylabel "Sum of rewards"'
          -s 'set y2label "Amount (Poured, Spilled)"'
          -s 'set y2range[0:1]'
          /tmp/dpl/dpl_sdump_s.dat u 1:4 w lp lt 1 lw 3 ct '"Sum of rewards"'
          , '0.3' ax x1y2 w l lt 2 ct '"Target amount"'
          /tmp/dpl/dpl_sdump_s.dat u 1:2 ax x1y2 w lp lt 4 ct '"Poured amount"'
          /tmp/dpl/dpl_sdump_s.dat u '1:($3*0.1)' ax x1y2 w lp lt 3 ct '"Spilled amount"'
          &''',
    '''qplot -x2 go
          -s 'set title "Which skill is used in what situation?"'
          -s 'set xlabel "Mouth size of source"'
          -s 'set ylabel "Viscosity parameter"'
          -s 'set xrange[*:*]; set yrange[-0.5:1.8]'
          /tmp/dpl/dpl_sdump_x.dat u 8:'($2==1 ? $11 : 1/0)':'(0.5+$13*10)'
            w p pt 6 lt 3 ps variable ct '"Shaking-A (Pt Size=Amount)"'
          /tmp/dpl/dpl_sdump_x.dat u 8:'($2==0 ? $11 : 1/0)':'(0.5+$13*10)'
            w p pt 6 lt 1 ps variable ct '"Tipping (Pt Size=Amount)"'
          &''',
    '''qplot -x2 go
          -s 'set title "When big errors happen?"'
          -s 'set xlabel "Mouth size of source"'
          -s 'set ylabel "Viscosity parameter"'
          -s 'set xrange[*:*]; set yrange[-0.5:1.8]'
          /tmp/dpl/dpl_sdump_x.dat u 8:'($1>50 && $2==1 && $15<-5 && $9<0.5 ? $11 : 1/0)':'(2+(-$15)/10)'
            w p pt 6 lt 3 ps variable ct '"Shaking-A (Pt Size=Amount)"'
          /tmp/dpl/dpl_sdump_x.dat u 8:'($1>50 && $2==1 && $15<-5 && $9>=0.5 ? $11 : 1/0)':'(2+(-$15)/10)'
            w p pt 12 lt 3 ps variable ct '"Shaking-A, Bounce (Pt Size=Amount)"'
          /tmp/dpl/dpl_sdump_x.dat u 8:'($1>50 && $2==0 && $15<-5 && $9<0.5 ? $11 : 1/0)':'(2+(-$15)/10)'
            w p pt 6 lt 1 ps variable ct '"Tipping (Pt Size=Amount)"'
          /tmp/dpl/dpl_sdump_x.dat u 8:'($1>50 && $2==0 && $15<-5 && $9>=0.5 ? $11 : 1/0)':'(2+(-$15)/10)'
            w p pt 12 lt 1 ps variable ct '"Tipping, Bounce (Pt Size=Amount)"'
          &''',
    #'''qplot -x2 go
          #-s 'set title "Sum of rewards v.s. Viscosity parameter"'
          #-s 'set ytic nomirror;set y2tic'
          #-s 'set xlabel "Num of episode"'
          #-s 'set ylabel "Sum of rewards"'
          #-s 'set y2label "Viscosity parameter"'
          #/tmp/dpl/dpl_sdump_s.dat u 1:4 w lp lt 1 lw 3 ct '"Sum of rewards"'
          #/tmp/dpl/dpl_sdump_x.dat u 1:11 ax x1y2 w lp lt 5 ct '"Viscosity parameter"'
          #&''',
          ##'0.3' w l lt 2 ct '"Target amount"'
          ##/tmp/dpl/dpl_sdump_s.dat u 1:2 w lp lt 4 ct '"Poured amount"'
          ##/tmp/dpl/dpl_sdump_s.dat u '1:($3*0.1)' w lp lt 3 ct '"Spilled amount"'
    #'''qplot -x2 go -s 'set title "dpl_log";set yrange [*:*];set y2tics;set ytics nomirror'
          #/tmp/dpl/dpl_log.dat u 87 w lp lw 3 ct '"R"'
          #/tmp/dpl/dpl_log.dat u 17 ax x1y2 w lp ct '"grasp height"'
          #/tmp/dpl/dpl_log.dat u 38 ax x1y2 w lp ct '"pour_x"'
          #/tmp/dpl/dpl_log.dat u 39 ax x1y2 w lp ct '"pour_z"' &''',
    '''f=/tmp/dpl/dpl_est.dat && qplot -x2 go -s 'set title "Estimated J v.s. Sum of rewards"'
          -s 'set yrange [-14:*];set xrange [0:300]'
          $f u 1:2 w lp lw 3 ct '"Sum of rewards"'
          $f u 1:3 w lp ct '"Estimated J"'  &''',
    '''f=/tmp/dpl/dpl_est.dat && qplot -x2 go -s 'set title "Estimated J v.s. Sum of rewards (log)"'
          -s 'set yrange [0.001:100] reverse;set xrange [0:300]'
          -s 'set logscale y'
          $f u 1:'(-$2)' w lp lw 3 ct '"Sum of rewards"'
          $f u 1:'(-$3)' w lp ct '"Estimated J"'  &''',
    #'''f=/tmp/dpl/dpl_pz_debug.dat && qplot -x2 go -s 'set title "pz debug"' $f u 2 w p $f u 3 w l $f u 4 w l $f u 5 w l $f u 6 w l $f u 7 w l $f u 8 w l $f u 9 w l $f u 10 w l $f u 11 w l &''',
    #'''qplot -x2 go -3d -s 'set zrange [-2.0:*];set xlabel "pour_x";set ylabel "pour_z";set title "action evaluation";set ticslevel 0' -cs 'u 2:3:4 w l' /tmp/dpl/act/act_eval-* &''',
    '''''',
    '''''',
    '''''',
    '''''',
    '''''',
    '''''',
    ]
  for cmd in commands:
    if cmd!='':
      cmd= ' '.join(cmd.splitlines()).replace('/tmp/dpl/',logdir)
      if qopt!='':
        cmd= cmd.replace('qplot -x2 go','qplot '+qopt)
        if cmd[-1]=='&':  cmd= cmd[:-1]
      print '###',cmd
      os.system(cmd)

  print '##########################'
  print '###Press enter to close###'
  print '##########################'
  raw_input()
  os.system('qplot -x2kill go')


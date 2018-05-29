#!/usr/bin/python
from core_tool import *
def Help():
  return '''Jacobian test.
  Usage: mikata.test.jacobian1'''

#t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6)
#{
  #using namespace Eigen;
  #typedef JacobiSVD<t_matrix> TSVD;
  #unsigned int svd_opt(ComputeThinU | ComputeThinV);
  #if(m.RowsAtCompileTime!=Dynamic || m.ColsAtCompileTime!=Dynamic)
    #svd_opt= ComputeFullU | ComputeFullV;
  #TSVD svd(m, svd_opt);
  #const typename TSVD::SingularValuesType &sigma(svd.singularValues());
  #typename TSVD::SingularValuesType sigma_inv(sigma.size());

  #for(long i=0; i<sigma.size(); ++i)
  #{
    #if(sigma(i) > tolerance)
      #sigma_inv(i)= 1.0/sigma(i);
    #else
      #sigma_inv(i)= 0.0;
  #}
  #return svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();

''' Weighted pseudo inverse.
  The code is based on numpy.linalg.pinv.
  a: Input matrix.
  w: Input weight.
def wpinv(a, w, rcond=1e-15):
  #a, wrap= _makearray(a)
  a_new= np.asarray(a)
  wrap= getattr(a, "__array_prepare__", a_new.__array_wrap__)
  a= a_new

  rcond = np.asarray(rcond)
  #if _isEmpty2d(a):
  if a.size==0 and np.product(a.shape[-2:])==0:
      res= np.empty(a.shape[:-2] + (a.shape[-1], a.shape[-2]), dtype=a.dtype)
      return wrap(res)
  a= a.conjugate()
  u,s,vt= la.svd(a, full_matrices=False)

  # discard small singular values
  cutoff= rcond[..., np.newaxis] * np.amax(s, axis=-1, keepdims=True)
  large= s > cutoff
  s= np.divide(1, s, where=large, out=s)  #Inverse of s
  s[~large]= 0

  res= np.dot(np.transpose(vt), np.multiply(s[..., np.newaxis], np.transpose(u)))
  return wrap(res)
'''

def Run(ct,*args):
  arm= 0
  J= ct.robot.J(ct.robot.Q(arm=arm),arm=arm)
  #J= Mat([[-0.00180975,  0.16397773,  0.04241847,  0.        ],
          #[ 0.0561604,   0.00528414,  0.00136692,  0.        ],
          #[ 0.       ,  -0.05618955, -0.14387079,  0.        ],
          #[ 0.       ,  -0.03220803, -0.03220803, -0.03220803],
          #[ 0.       ,   0.99948119,  0.99948119,  0.99948119],
          #[ 1.       ,   0.        ,  0.        ,  0.        ]])

  print 'J=',J

  W= np.diag([1.0,1.0,1.0, 0.01,0.01,0.01])

  vx0= [0,1,0, 0,0,0]
  dq1= la.pinv(J)*MCVec(vx0)
  dq2= la.pinv(J[:3,])*MCVec(vx0[:3])
  dq3= la.pinv(J[:3,])*MCVec(vx0[:3]) + 0.1*la.pinv(J)*MCVec(vx0)
  dq4= la.pinv(W*J)*W*MCVec(vx0)
  vx1= np.dot(J,dq1)
  vx2= np.dot(J,dq2)
  vx3= np.dot(J,dq3)
  vx4= np.dot(J,dq4)

  print 'dq1=',ToList(dq1)
  print 'dq2=',ToList(dq2)
  print 'dq3=',ToList(dq3)
  print 'dq4=',ToList(dq4)
  print 'vx0=',ToList(vx0)
  print 'vx1=',ToList(vx1)
  print 'vx2=',ToList(vx2)
  print 'vx3=',ToList(vx3)
  print 'vx4=',ToList(vx4)


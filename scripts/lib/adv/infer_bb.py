#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer a bounding box.
  Usage: adv.infer_bb OBJ
    OBJ: ID of object.  e.g. 'b1'
  '''
def Run(ct,*args):
  obj= args[0]

  if ct.HasAttr(obj,'bound_box'):
    pass  #Do nothing if already exists
  else:
    ##Get a bounding box
    res= True
    if not ct.HasAttr(obj,'grab_primitives'):
      raise Exception('adv.infer_bb requires the target object {obj} must have grab_primitives attribute.'.format(obj=obj))
    bpmax= [0.0,0.0,0.0]
    bpmin= [0.0,0.0,0.0]
    for prm in ct.GetAttr(obj,'grab_primitives'):
      if prm['kind']=='pkCylinder':
        w= prm['width']
        p1= Vec(prm['p1'])
        p2= Vec(prm['p2'])
        h= Norm(p2-p1)
        axis= np.cross([0.0,0.0,1.0],p2-p1)
        if Norm(axis)>1.0e-6:
          angle= GetAngle([0.0,0.0,1.0],p2-p1)
          R= RFromAxisAngle(Normalize(axis),angle)
        else:
          R= Eye()
        c= 0.5*(p1+p2)
        n_div= 40
        for i in range(n_div):
          angle= 2.0*math.pi*float(i)/float(n_div)
          p= [0.5*w*math.cos(angle),0.5*w*math.sin(angle),0.5*h]
          p= np.dot(R,p) + c
          bpmax= [max(d) for d in zip(bpmax,p)]
          bpmin= [min(d) for d in zip(bpmin,p)]
          p= [0.5*w*math.cos(angle),0.5*w*math.sin(angle),-0.5*h]
          p= np.dot(R,p) + c
          bpmax= [max(d) for d in zip(bpmax,p)]
          bpmin= [min(d) for d in zip(bpmin,p)]
      elif prm['kind']=='pkCube':
        x_center= Vec(prm['x_center'])
        dims= Vec(prm['dims'])
        corners= [[1.,1.,1.],[-1.,1.,1.],[-1.,-1.,1.],[1.,-1.,1.],
                  [1.,1.,-1.],[-1.,1.,-1.],[-1.,-1.,-1.],[1.,-1.,-1.]]
        for lp in corners:
          p= Transform(x_center, [0.5*dims[d]*lp[d] for d in range(3)])
          bpmax= [max(d) for d in zip(bpmax,p)]
          bpmin= [min(d) for d in zip(bpmin,p)]
    #if ct.HasAttr(obj,'l_p_pour_e_set'):
      #l_p_pour_e_set= ct.GetAttr(obj,'l_p_pour_e_set')
      #for p in l_p_pour_e_set:
        #bpmax= [max(d) for d in zip(bpmax,p)]
        #bpmin= [min(d) for d in zip(bpmin,p)]
    center= list(0.5*(Vec(bpmin)+Vec(bpmax)))+[0.0,0.0,0.0,1.0]
    dim= [d[1]-d[0] for d in zip(bpmin,bpmax)]
    ct.SetAttr(obj,'bound_box', {'center':center,'dim':dim})


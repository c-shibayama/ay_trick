#!/usr/bin/python
from core_tool import *
import tf
def Help():
  return '''One time tf listening.
    Useful when obtaining static transformation.
  Usage: tf_once TRG_FRAME, SRC_FRAME [, ATTR_KEYS]
    TRG_FRAME, SRC_FRAME: Target and source frame ids.
    ATTR_KEYS: Attribute keys to store; e.g. ('left_hand_camera','lx') (default:None). '''
def Run(ct,*args):
  trg_frame,src_frame= args[:2]
  attr_keys= args[2] if len(args)>2 else None
  if attr_keys not in (None,[]) and ct.HasAttr(*attr_keys):
    return ct.GetAttr(*attr_keys)
  trans_rot= TfOnce(trg_frame, src_frame, time_out=4.0)
  if attr_keys is not None:
    ct.SetAttr(*(attr_keys + (trans_rot,)))
  return trans_rot


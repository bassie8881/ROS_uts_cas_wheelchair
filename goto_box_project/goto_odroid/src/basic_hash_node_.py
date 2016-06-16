#!/usr/bin/env python

import rospy
from pyhash.srv import Hash,HashResponse

import os
import numpy as np

def Pyhash_basic():
  def hash_svc(req): # hashing callback
    if len(req.key) == nd_:
      hash_ = (np.round(np.asarray(req.key)*sc_) - lb_) * P_
      return HashResponse(LUT_[mod_wrap(int(np.sum(hash_)))])
    else:
      print "Error: |key| != %d" % nd_; return HashResponse()

  def mod_wrap(idx):
    out_ = idx;
    while out_ < 0: out_ = out_ + idxmax_
    while out_ > idxmax_: out_ = out_ - idxmax_
    return out_
      
  def proc_raw_LUT(lut): # parse LUT
    lutsz_ = int(np.amax(lut[:,0])) + 1; lut_ = []
    for i in range(0,lutsz_): lut_.append([])
    for i in range(0,lut.shape[0]): lut_[int(lut[i,0])] = lut[i,1:]
    return lut_,lutsz_
  
  def proc_raw_LUTargs(args): # parse LUT metadata
    pidx_ = 2 + args[1]
    return args[0],args[1],args[2:pidx_],args[pidx_:]
    
  # init
  rospy.init_node('pyhash_basic_node', anonymous=True)
  lutfp_ = rospy.get_param("~lut_fp"); lutargsfp_ = rospy.get_param("~lut_args_fp")
  if os.path.isfile(lutfp_) and os.path.isfile(lutargsfp_):
    # parse LUT files
    LUT_,idxmax_ = proc_raw_LUT(np.loadtxt(lutfp_,delimiter=','))
    sc_,nd_,lb_,P_ = proc_raw_LUTargs(np.loadtxt(lutargsfp_,delimiter=','))
    print "LUT and args imported"
    # ROS service server, spin
    srv_topic_ = '/pyhash/' + rospy.get_param("~lut")
    srv_ = rospy.Service(srv_topic_, Hash, hash_svc)
    rospy.spin()    
  else:
    print "Failed to import LUT files, please check lut arg"
    
if __name__ == '__main__':
  Pyhash_basic()

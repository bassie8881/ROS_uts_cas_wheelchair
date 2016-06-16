#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from pyhash.srv import Hash

def JT_inv():
 
  def twist_cb(msg): # twist -> joy callback
    res_ = hash_client_([msg.linear.x, msg.angular.z])
    jmsg_ = Joy(); jmsg_.axes = res_.bin
    jmsg_.header.stamp = rospy.get_rostime(); pub_.publish(jmsg_)
  
  rospy.init_node('jt_inv_node', anonymous=True) # init
  # hash client
  hashtopic_ = rospy.get_param("~hash_topic")
  rospy.wait_for_service(hashtopic_); hash_client_ = rospy.ServiceProxy(hashtopic_, Hash)
  # twist in, joy out, spin
  rospy.Subscriber("/cmd_vel", Twist, twist_cb)
  pub_ = rospy.Publisher('/wc_joy', Joy, queue_size=1)
  rospy.spin()

if __name__ == '__main__':
  JT_inv()

#!/usr/bin/env python3  
import roslib

import rospy
import tf

import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return qx, qy, qz, qw

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    qx, qy, qz, qw = get_quaternion_from_euler(3.14,0,0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.72),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "camera_struct",
                         "world")
        br.sendTransform((0.0, 0.0, 0.0),
                         (qx, qy, qz, qw),
                         rospy.Time.now(),
                         "usb_cam",
                         "camera_struct")
        rate.sleep()

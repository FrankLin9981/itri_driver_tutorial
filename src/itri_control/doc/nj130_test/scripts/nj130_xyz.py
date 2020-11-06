#!/usr/bin/env python
import rospy
import time
import cv2
import tf
import numpy as np
from geometry_msgs.msg import PointStamped


def getTF():
   listener = tf.TransformListener()
   listener.waitForTransform("/left_tf/world", "/left_tf/hand", rospy.Time(0), rospy.Duration(3.0))
   listener.waitForTransform("/right_tf/world", "/right_tf/hand", rospy.Time(0), rospy.Duration(3.0))
   
   r = rospy.Rate(5)
   while not rospy.is_shutdown():
      (trans, rot) = listener.lookupTransform("/left_tf/hand", "/left_tf/world", rospy.Time(0))
      T_lw = listener.fromTranslationRotation(trans, rot)
      (trans, rot) = listener.lookupTransform("/right_tf/world", "/right_tf/hand", rospy.Time(0))
      T_wr = listener.fromTranslationRotation(trans, rot)
      T_lr = np.array(
         [[-1., 0.,  0.,  4.],
         [0., -1.,  0.,  0.],
         [0.,  0.,  1.,  0.],
         [0.,  0.,  0.,  1.]])

      print(T_lw.dot(T_lr.dot(T_wr)))
      print()
      r.sleep()

def getXYZ():
   p = PointStamped()
   p.header.frame_id = "/left_tf/flange"
   p.point.x = 0.0
   p.point.y = 0.0
   p.point.z = 0.0

   listener = tf.TransformListener()
   
   r = rospy.Rate(10) # 10hz 
   while not rospy.is_shutdown():
      listener.waitForTransform("/left_tf/world", "/left_tf/link6", rospy.Time(0), rospy.Duration(3.0))
      p_new = listener.transformPoint("/left_tf/world", p)
      rospy.loginfo(p_new)
      r.sleep()

if __name__ == '__main__':
   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   rospy.init_node('calc_tf', anonymous=True)

   try:
      getTF()
      # getXYZ()
   except rospy.ROSInterruptException:
      pass
#!/usr/bin/env python
import rospy
import sys
import cv2
import tf
import copy
import moveit_commander
import numpy as np
from move_group_api import *
from math import pi
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global cur_pc, cur_rgb

def callback_point(data):
   global cur_pc
   assert isinstance(data, PointCloud2)

   gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
   cur_pc = list(gen)

def callback_rgb(data):
   global cur_rgb
   assert isinstance(data, Image)

   bridge = CvBridge()
   try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
   except CvBridgeError as e:
      print(e)

   cur_rgb = cv_image.copy()

def detectHole():
   global cur_rgb, cur_pc

   original = cur_rgb.copy()
   output = cur_rgb.copy()
   gray = cv2.cvtColor(cur_rgb, cv2.COLOR_BGR2GRAY)
   gray_blurred = cv2.blur(gray, (3, 3))
   # detect circles in the image
   circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 20,
                              param1 = 100, param2 = 30, minRadius = 1, maxRadius = 20) 
   # ensure at least some circles were found
   if circles is not None:
      # convert the (x, y) coordinates and radius of the circles to integers
      circles = np.round(circles[0, :]).astype("int")
      # loop over the (x, y) coordinates and radius of the circles
      for (x, y, r) in circles:
         # draw the circle in the output image, then draw a rectangle
         # corresponding to the center of the circle
         cv2.circle(output, (x, y), r, (0, 255, 0), 2)
         cv2.rectangle(output, (x - 2, y - 2), (x + 2, y + 2), (0, 128, 255), -1)
   
   # show the output image
   cv2.imwrite('/home/frank/Desktop/data/circles_detected.png', np.hstack([original, output]))

   return circles

if __name__ == '__main__':
   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   rospy.init_node('PiH', anonymous=True)

   moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
   robot = moveit_commander.robot.RobotCommander()
   arm_group = moveit_commander.MoveGroupCommander("manipulator")

   # Get transformation matrix between world and kinect image frame
   listener = tf.TransformListener()
   listener.waitForTransform("/world", "/d415_depth_optical_frame", rospy.Time(0), rospy.Duration(3.0))
   listener.waitForTransform("/world", "/flange", rospy.Time(0), rospy.Duration(3.0))

   ##################### Ground truth #####################
   go_to_pose_goal(arm_group, [0.090749, 0.4, 0.5, pi, 0, pi/2], w=True)
   go_to_pose_goal(arm_group, [0.090749, 0.5, 0.3, pi, 0, pi/2], w=True)

   ##################### Vision-guided #####################
   # fs = cv2.FileStorage("/home/frank/ws_moveit/src/handeye_calib_camodocal/example/CalibratedTransform.yml", cv2.FileStorage_READ)
   # T_cali = fs.getNode("ArmTipToCameraTransform").mat()
   
   # go_to_pose_goal(arm_group, [0.090749, 0.4, 0.5, pi, 0, pi/2], w=True)
   # data = rospy.wait_for_message("/d415/color/image_raw", Image, timeout=1)
   # callback_rgb(data)
   # data = rospy.wait_for_message("/d415/depth/points", PointCloud2, timeout=1)
   # callback_point(data)
   
   # holes = detectHole()
   # print(holes)
   # (x, y, r) = holes[0]

   # (trans, rot) = listener.lookupTransform("/world", "/flange", rospy.Time(0))
   # T_flange = listener.fromTranslationRotation(trans, rot)
   # T = np.dot(T_flange, T_cali)
   # (trans, rot) = listener.lookupTransform("/world", "/d415_depth_optical_frame", rospy.Time(0))
   # T = listener.fromTranslationRotation(trans, rot)

   # p = np.dot(T, cur_pc[x+y*640] + (1,))
   # print("Estimated hole: {} {} {}".format(p[0], p[1], p[2]))

   # go_to_pose_goal(arm_group, [p[0], p[1], 0.3, pi, 0, pi/2], w=True)

   waypoints = []
   wpose = arm_group.get_current_pose().pose
   wpose.position.z = 0.172
   waypoints.append(copy.deepcopy(wpose))
   wpose.position.z = 0.3
   waypoints.append(copy.deepcopy(wpose))
   wpose.position.z = 0.172
   waypoints.append(copy.deepcopy(wpose))
   wpose.position.z = 0.3
   waypoints.append(copy.deepcopy(wpose))
   wpose.position.z = 0.172
   waypoints.append(copy.deepcopy(wpose))
   wpose.position.z = 0.3
   waypoints.append(copy.deepcopy(wpose))

   plan, _ = plan_cartesian_path(arm_group, waypoints)
   execute_plan(arm_group, plan, w=True)

   go_home(arm_group, w=True)
   
   moveit_commander.roscpp_initializer.roscpp_shutdown()

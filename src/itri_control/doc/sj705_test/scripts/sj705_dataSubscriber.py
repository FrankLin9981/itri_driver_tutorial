#!/usr/bin/env python
import rospy
import time
import csv
import cv2
import tf
import numpy as np
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs import point_cloud2
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

def callback_js_left(data):   
    global w_js_left
    # Get joint states
    values = []
    values.append(data.header.stamp.secs + data.header.stamp.nsecs*10**(-9))
    values.extend(data.position)
    w_js_left.writerow(values)

def callback_js_right(data): 
    global w_js_right
    # Get joint states
    values = []
    values.append(data.header.stamp.secs + data.header.stamp.nsecs*10**(-9))
    values.extend(data.position)
    w_js_right.writerow(values)

def getData():    
    global w_js_left, w_js_right

    # CSV stream

    csvfile = open('/home/frank/Desktop/data/js_left.csv', 'w+')
    w_js_left = csv.writer(csvfile)
    w_js_left.writerow(['time', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'])

    csvfile = open('/home/frank/Desktop/data/js_right.csv', 'w+')
    w_js_right = csv.writer(csvfile)
    w_js_right.writerow(['time', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'jont7'])

    cnt = -1

    rospy.Subscriber("/left/right_arm_controller/joint_states", JointState, callback_js_left)
    rospy.Subscriber("/right/right_arm_controller/joint_states", JointState, callback_js_right)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('get_data', anonymous=True)

    try:
        getData()
    except rospy.ROSInterruptException:
        pass

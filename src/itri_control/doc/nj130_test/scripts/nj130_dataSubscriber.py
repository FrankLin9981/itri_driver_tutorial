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


def callback_ft_raw(data):
    global w_comp
    # Get force/torque data
    values = []
    values.append(data.header.stamp.secs + data.header.stamp.nsecs*10**(-9))
    values.append(data.wrench.force.x)
    values.append(data.wrench.force.y)
    values.append(data.wrench.force.z)
    values.append(data.wrench.torque.x)
    values.append(data.wrench.torque.y)
    values.append(data.wrench.torque.z)
    w_raw.writerow(values)

def callback_ft_comp(data):
    global w_comp
    # Get force/torque data
    values = []
    values.append(data.header.stamp.secs + data.header.stamp.nsecs*10**(-9))
    values.append(data.wrench.force.x)
    values.append(data.wrench.force.y)
    values.append(data.wrench.force.z)
    values.append(data.wrench.torque.x)
    values.append(data.wrench.torque.y)
    values.append(data.wrench.torque.z)
    w_comp.writerow(values)

    # rospy.loginfo(values)

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

def callback_ls(data): 
    global w_arms_pose, w_obj_pose
    # Get link states
    now = rospy.get_rostime()
    p_left = data.pose[data.name.index('left_manipulator::hand')]
    p_right = data.pose[data.name.index('right_manipulator::hand')]
    p_obj = data.pose[data.name.index('l_shape::link1')]
    arms = []
    obj = []
    arms.append(now.secs + now.nsecs*10**(-9))
    arms.extend([p_left.position.x, p_left.position.y, p_left.position.z,
                        p_left.orientation.x, p_left.orientation.y, p_left.orientation.z, p_left.orientation.w])
    arms.extend([p_right.position.x, p_right.position.y, p_right.position.z,
                        p_right.orientation.x, p_right.orientation.y, p_right.orientation.z, p_right.orientation.w])
    obj.append(now.secs + now.nsecs*10**(-9))
    obj.extend([p_obj.position.x, p_obj.position.y, p_obj.position.z,
                        p_obj.orientation.x, p_obj.orientation.y, p_obj.orientation.z, p_obj.orientation.w])
    w_arms_pose.writerow(arms)
    w_obj_pose.writerow(obj)

def getData():    
    global w_raw, w_comp, w_js_left, w_js_right, w_arms_pose, w_obj_pose

    # CSV stream
    csvfile = open('/home/frank/Desktop/data/ft_raw.csv', 'w+')
    w_raw = csv.writer(csvfile)
    w_raw.writerow(['time', 'force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z'])

    csvfile = open('/home/frank/Desktop/data/ft_comp.csv', 'w+')
    w_comp = csv.writer(csvfile)
    w_comp.writerow(['time', 'force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z'])

    csvfile = open('/home/frank/Desktop/data/js_left.csv', 'w+')
    w_js_left = csv.writer(csvfile)
    w_js_left.writerow(['time', 'finger_joint1', 'finger_joint2', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])

    csvfile = open('/home/frank/Desktop/data/js_right.csv', 'w+')
    w_js_right = csv.writer(csvfile)
    w_js_right.writerow(['time', 'finger_joint1', 'finger_joint2', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])

    csvfile = open('/home/frank/Desktop/data/arms_pose.csv', 'w+')
    w_arms_pose = csv.writer(csvfile)
    w_arms_pose.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'w', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'w'])

    csvfile = open('/home/frank/Desktop/data/obj_pose.csv', 'w+')
    w_obj_pose = csv.writer(csvfile)
    w_obj_pose.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'w'])

    cnt = -1

    rospy.Subscriber("/right_manipulator/ft_raw", WrenchStamped, callback_ft_raw)
    rospy.Subscriber("/right_manipulator/ft_compensated", WrenchStamped, callback_ft_comp)
    rospy.Subscriber("/left_manipulator/joint_states", JointState, callback_js_left)
    rospy.Subscriber("/right_manipulator/joint_states", JointState, callback_js_right)
    # rospy.Subscriber("/gazebo/link_states", LinkStates, callback_ls)


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

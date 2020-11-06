#!/usr/bin/env python
import rospy
import time
import csv
import cv2
import tf
import numpy as np
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
    
def callback_rgb(data):
    global out
    assert isinstance(data, Image)
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    out.write(cv_image)

def callback_js(data):   
    global w_jPos
    # Get joint states
    values = []
    values.append(data.header.stamp.secs + data.header.stamp.nsecs*10**(-9))
    values.extend(data.position)
    w_jPos.writerow(values)

def callback_ls(data): 
    global w_arm_pose, w_obj_pose
    # Get link states
    now = rospy.get_rostime()
    p_arm = data.pose[data.name.index('itri_ar607::hand')]
    p_obj = data.pose[data.name.index('shaft2::link')]
    arm = []
    obj = []
    arm.append(now.secs + now.nsecs*10**(-9))
    arm.extend([p_arm.position.x, p_arm.position.y, p_arm.position.z,
                        p_arm.orientation.x, p_arm.orientation.y, p_arm.orientation.z, p_arm.orientation.w])
    obj.append(now.secs + now.nsecs*10**(-9))
    obj.extend([p_obj.position.x, p_obj.position.y, p_obj.position.z,
                        p_obj.orientation.x, p_obj.orientation.y, p_obj.orientation.z, p_obj.orientation.w])
    w_arm_pose.writerow(arm)
    w_obj_pose.writerow(obj)

def getData():
    global w_jPos, w_arm_pose, w_obj_pose, writer, out, cnt

    # CSV stream
    csvfile = open('/home/frank/Desktop/data/jPos.csv', 'w+')
    w_jPos = csv.writer(csvfile)
    w_jPos.writerow(['time', 'finger_joint1', 'finger_joint2', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])

    csvfile = open('/home/frank/Desktop/data/arm_pose.csv', 'w+')
    w_arm_pose = csv.writer(csvfile)
    w_arm_pose.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'w'])

    csvfile = open('/home/frank/Desktop/data/obj_pose.csv', 'w+')
    w_obj_pose = csv.writer(csvfile)
    w_obj_pose.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'w'])

    cnt = -1
    
    # Video stream
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter('/home/frank/Desktop/data/output.mp4', fourcc, 20.0, (640, 480))

    rospy.Subscriber("/joint_states", JointState, callback_js)
    # rospy.Subscriber("/gazebo/link_states", LinkStates, callback_ls)
    # rospy.Subscriber("/d415/color/image_raw", Image, callback_rgb)

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
        out.release()
        pass

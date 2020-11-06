#!/usr/bin/env python
import rospy
import math
import csv
from std_msgs.msg import Float64

def visual_servoing():
    rospy.init_node('visual_servoing', anonymous=True)
    pub = rospy.Publisher('/ar607/d415_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(4)

    rospy.sleep(1)

    csvfile = open('/home/frank/Desktop/data/cam_cmd.csv', 'w+')
    w_cam_cmd = csv.writer(csvfile)
    w_cam_cmd.writerow(['time', 'cmd'])

    t = rospy.get_rostime()
    pub.publish(0.7854)
    w_cam_cmd.writerow([t, 0.7854])

    # while not rospy.is_shutdown():
    #     t = rospy.get_rostime()
    #     cmd = 0.7854 * math.sin(rospy.get_time())
    #     pub.publish(cmd)
    #     w_cam_cmd.writerow([t, cmd])
    #     rate.sleep()

if __name__ == '__main__':
    try:
        visual_servoing()
    except rospy.ROSInterruptException:
        pass
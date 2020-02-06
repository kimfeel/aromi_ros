#!/usr/bin/env python
## PoseWithCovarianceStamped to PoseStamped

from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
import sys
import json
from collections import deque

import time


def callback(data):
        pose = PoseStamped()
	pose.header = data.header
	pose.pose   = data.pose.pose
       
        pose.header.seq = data.header.seq + 1
        pose.header.frame_id = "camera_odom_frame"
        #pose.header.stamp = rospy.Time.now()
        pose.header.stamp = data.header.stamp
        
        pub.publish(pose)


if __name__ == '__main__':
        # Initializing node
        rospy.init_node('pose_publisher')

        pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=100)
        #msg = Odometry()

        # Subscription to the required odom topic (edit accordingly)
	rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, callback)
        #rospy.Subscriber('/camera/odom/sample', Odometry, callback)

        rate = rospy.Rate(30)  # 30hz

        try:
                while not rospy.is_shutdown():
                    # rospy.spin()
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass

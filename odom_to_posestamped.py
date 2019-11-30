#!/usr/bin/env python
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

        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.position.z = float(data.pose.pose.position.z)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)

        pose.header.seq = data.header.seq + 1
        pose.header.frame_id = "camera_odom_frame"
        pose.header.stamp = rospy.Time.now()
        pose.header.stamp = data.header.stamp
        
        pub.publish(pose)


if __name__ == '__main__':
        # Initializing node
        rospy.init_node('pose_publisher')

        pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        msg = Odometry()

        # Subscription to the required odom topic (edit accordingly)
        msg = rospy.Subscriber('/camera/odom/sample', Odometry, callback)

        rate = rospy.Rate(30)  # 30hz

        try:
                while not rospy.is_shutdown():
                    # rospy.spin()
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass

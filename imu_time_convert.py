## IMU time convert

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
timestamp_secs = 0
timestamp_nsecs = 0


def callback(data):
	global timestamp_secs, timestamp_nsecs
        odom = Odometry()

        odom.pose.pose.position.x = float(data.pose.pose.position.x)
        odom.pose.pose.position.y = float(data.pose.pose.position.y)
        odom.pose.pose.position.z = float(data.pose.pose.position.z)
        odom.pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        odom.pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        odom.pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        odom.pose.pose.orientation.w = float(data.pose.pose.orientation.w)

	odom.pose.covariance = list(data.pose.covariance) 	# make turfle to list to handle matrix
	odom.pose.covariance[14] = 10 				# not to make Covariance specified diagonal matrix is zero
	odom.pose.covariance[21] = 10 				# not to make Covariance specified diagonal matrix is zero
	odom.pose.covariance[28] = 10 				# not to make Covariance specified diagonal matrix is zero

#	print(odom.pose.covariance)
#	for i in range(0,35) :
#		odom.pose.covariance[i] = data.pose.covariance[i]
#	odom.pose.covariance = [99999, 0, 0, 0, 0, 0,
#				0, 99999, 0, 0, 0, 0,
#				0, 0, 99999, 0, 0, 0,
#				0, 0, 0, 99999, 0, 0,
#				0, 0, 0, 0, 99999, 0,
#				0, 0, 0, 0, 0, 99999]

        odom.header.seq = data.header.seq + 1
        odom.header.frame_id = "map"
        #data.header.stamp = rospy.Time.now()
        odom.header.stamp.secs = timestamp_secs			# time match between sensors, Used RealSense Time #data.header.stamp
        odom.header.stamp.nsecs = timestamp_nsecs		# time match between sensors, Used RealSense Time #data.header.stamp
        
        pub.publish(odom)


def vo_callback(data):
	global timestamp_secs, timestamp_nsecs
	timestamp_secs = data.header.stamp.secs
	timestamp_nsecs = data.header.stamp.nsecs

if __name__ == '__main__':
        # Initializing node
        rospy.init_node('pose_publisher')

        pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        # Subscription to the required odom topic (edit accordingly)
        rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback)
	rospy.Subscriber('/camera/odom/sample', Odometry, vo_callback)

        rate = rospy.Rate(30)  # 30hz

        try:
                while not rospy.is_shutdown():
                    # rospy.spin()
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass

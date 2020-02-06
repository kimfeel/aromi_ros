#!/usr/bin/env python
## imu_hector_throttler

from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy, Imu
import sys
import json
from collections import deque

import threading

class Throttler:
	def __init__(self):
		self.recent_hector_odom = None
		self.recent_imu = None

		self.pub_hector_odom = None
		self.pub_imu = None

		self.lock = threading.Lock()

	def try_to_publish(self):
		self.lock.acquire()
		if (self.recent_hector_odom is not None) and \
		   (self.recent_imu is not None):
			#print("recent_imu: {}".format(self.recent_imu))
			#print("recent_hector_odom: {}".format(self.recent_hector_odom))
			self.pub_hector_odom.publish(self.recent_hector_odom)
			self.pub_imu.publish(self.recent_imu)

			self.recent_hector_odom = None
			self.recent_imu = None
		self.lock.release()

	def on_poseupdate(self, data):
		self.lock.acquire()
	        self.recent_hector_odom = data

		self.lock.release()

		self.try_to_publish()


	def on_imu(self, data):
		self.lock.acquire()
#		print("on_imu: {}".format(type(data)))
		self.recent_imu = data
		self.lock.release()

		self.try_to_publish()

if __name__ == '__main__':
        # Initializing node
        rospy.init_node('pose_publisher')

	th = Throttler()

        th.pub_hector_odom = rospy.Publisher('/slam_data', PoseStamped, queue_size=100)
        th.pub_imu         = rospy.Publisher('/imu_data', Imu, queue_size=100)

        # Subscription to the required odom topic (edit accordingly)
        rospy.Subscriber('/slam_out_pose', PoseStamped, th.on_poseupdate)
	#rospy.Subscriber('/camera/odom/sample', Odometry, th.on_cam_odom)
	rospy.Subscriber('/mavros/imu/data', Imu, th.on_imu)
        #rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback)
#	rospy.Subscriber('/camera/odom/sample', Odometry, vo_callback)
#	rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)


        rate = rospy.Rate(30)  # 30hz

        try:
                while not rospy.is_shutdown():
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass

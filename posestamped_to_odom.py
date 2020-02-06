#!/usr/bin/env python
## PoseStamped to Odometry

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
		self.recent_cam_odom = None
		#self.recent_imu = None

		self.pub_hector_odom = None
		self.pub_cam_odom = None
		#self.pub_imu = None

		self.lock = threading.Lock()

	def try_to_publish(self):
		self.lock.acquire()
		if (self.recent_hector_odom is not None) and \
		   (self.recent_cam_odom is not None): #and \ 
		   #(self.recent_imu is not None):
			#print("publishing: \n{}, \n{}, \n{}".format(self.recent_hector_odom.header.stamp, self.recent_cam_odom.header.stamp, self.recent_imu.header.stamp))
			#print("recent_imu: {}".format(self.recent_imu))
			#print("recent_hector_odom: {}".format(self.recent_hector_odom))
			self.pub_hector_odom.publish(self.recent_hector_odom)
			self.pub_cam_odom.publish(self.recent_cam_odom)
			#self.pub_imu.publish(self.recent_imu)

			self.recent_hector_odom = None
			self.recent_cam_odom = None
			#self.recent_imu = None
		self.lock.release()

	def on_poseupdate(self, data):
		self.lock.acquire()
	        self.recent_hector_odom = Odometry()

		self.recent_hector_odom.header = data.header
		self.recent_hector_odom.pose   = data.pose
		self.recent_hector_odom.pose.covariance = list(data.pose.covariance)

		self.recent_hector_odom.pose.covariance[14] = 99999 		# ONLY x,y position and z needed for 2D Hector SLAM
		self.recent_hector_odom.pose.covariance[21] = 99999 		# Large number for z posiion and x, y rotation value going to be ignored
		self.recent_hector_odom.pose.covariance[28] = 99999 		# not to make Covariance specified diagonal matrix is zero
		self.lock.release()

		self.try_to_publish()

	def on_cam_odom(self, data):
		self.lock.acquire()
		self.recent_cam_odom = data
		#self.recent_cam_odom.header = data.header
		#self.recent_cam_odom.pose   = data.pose
		#self.recent_cam_odom = copy.deepcopy(data)
		self.lock.release()

		self.try_to_publish()

	def on_imu(self, data):
		self.lock.acquire()
#		print("on_imu: {}".format(type(data)))
		self.recent_imu = data
		#self.recent_imu = copy.deepcopy(data)
		self.lock.release()

		self.try_to_publish()

if __name__ == '__main__':
        # Initializing node
        rospy.init_node('odom_publisher')

	th = Throttler()

        th.pub_hector_odom = rospy.Publisher('/odom', Odometry, queue_size=100)
        th.pub_cam_odom    = rospy.Publisher('/vo', Odometry, queue_size=100)
        #th.pub_imu         = rospy.Publisher('/imu_data', Imu, queue_size=100)

        #pub = rospy.Publisher('/odom', Odometry, queue_size=1)
#	pub_imu = rospy.Publisher('/imu_data', Imu, queue_size=1)

        # Subscription to the required odom topic (edit accordingly)
        rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, th.on_poseupdate)
	rospy.Subscriber('/camera/odom/sample', Odometry, th.on_cam_odom)
	#rospy.Subscriber('/mavros/imu/data', Imu, th.on_imu)
        #rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback)
#	rospy.Subscriber('/camera/odom/sample', Odometry, vo_callback)
#	rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)


        rate = rospy.Rate(30)  # 30hz

        try:
                while not rospy.is_shutdown():
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass

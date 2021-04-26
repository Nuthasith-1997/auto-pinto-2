#!/usr/bin/env python

import rospy
#import tf2_ros

from geometry_msgs.msg import PoseStamped #, TransformStamped
from nav_msgs.msg import Odometry

class wheelOdom2Vision:
	def __init__(self):
		rospy.init_node('wo_2_vision')
		print "Node 'wo_2_vision' has initialized."

		rospy.Subscriber('/mavros/wheel_odometry/odom', Odometry, self.wo_cb)

		self.counter = 0

		self.pub_vision = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
		self.vision = PoseStamped()

		self.vision.header.seq = self.counter
		self.vision.header.stamp = rospy.Time.now()
		self.vision.header.frame_id = 'odom'
		self.vision.pose.position.x = 0
		self.vision.pose.position.y = 0
		self.vision.pose.position.z = 0
		self.vision.pose.orientation.x = 0
		self.vision.pose.orientation.y = 0
		self.vision.pose.orientation.z = 0
		self.vision.pose.orientation.w = 1
		'''
		self.tf_bc = tf2_ros.TransformBroadcaster()
		self.tf = geometry_msgs.msg.TransformStamped()

		self.tf.header.seq = self.counter
		self.tf.header.stamp = rospy.Time.now()
		self.tf.header.frame_id = 'odom'
		self.tf.child_frame_id = 'base_footprint'
		self.tf.transform.translation.x = 0
		self.tf.transform.translation.y = 0
		self.tf.transform.translation.z = 0
		self.tf.transform.rotation.x = 0
		self.tf.transform.rotation.y = 0
		self.tf.transform.rotation.z = 0
		self.tf.transform.rotation.w = 1
		'''
	def wo_cb(self, msg):
		self.vision.header.seq = self.counter
		self.vision.header.stamp = rospy.Time.now()
		self.vision.pose = msg.pose.pose
		#self.vision.pose.covariance = msg.pose.covariance
		'''
		self.tf.header.stamp = rospy.Time.now()
		self.tf.header.seq = self.counter
		self.tf.transform = msg.pose.pose
		'''
		self.pub_vision.publish(self.vision)
		#self.tf_bc.sendTransform(self.tf)
		self.counter += 1

if __name__ == '__main__':
	wheelOdom2Vision()
	rospy.spin()
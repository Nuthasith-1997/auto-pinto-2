#!/usr/bin/env python

import rospy

from tf.transformations import quaternion_from_euler
from tf2_ros import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped

class createTf:
	def __init__(self, parent, child, x, y, z, r, p, yaw):
		self.static_transformStamped = TransformStamped()

		quat = quaternion_from_euler(r, p, yaw)
		th_x = quat[0]
		th_y = quat[1]
		th_z = quat[2]
		th_w = quat[3]

		self.static_transformStamped.header.stamp = rospy.Time.now()
		self.static_transformStamped.header.frame_id = parent
		self.static_transformStamped.child_frame_id = child

		self.static_transformStamped.transform.translation.x = x
		self.static_transformStamped.transform.translation.y = y
		self.static_transformStamped.transform.translation.z = z

		self.static_transformStamped.transform.rotation.x = th_x
		self.static_transformStamped.transform.rotation.y = th_y
		self.static_transformStamped.transform.rotation.z = th_z
		self.static_transformStamped.transform.rotation.w = th_w

if __name__ == '__main__':
	rospy.init_node('static_tf_node')
	print "Node 'static_tf_node' has initialized."

	broadcaster = StaticTransformBroadcaster()

	base_link = createTf("base_footprint", "base_link", 0, 0, 0.05415, 0, 0, 0)
	base_scan = createTf("base_link", "base_scan", 0, 0, 0.23085, 0, 0, 0)
	wheel_left_link = createTf("base_link", "wheel_left_link", -0.09, 0.152, 0, -1.5708, 0, 0)
	wheel_right_link = createTf("base_link", "wheel_right_link", -0.09, -0.152, 0, -1.5708, 0, 0)
	imu_link = createTf("base_link", "imu_link", 0.183, 0.051882, 0.14285, 0, 0, 0)

	broadcaster.sendTransform([base_link.static_transformStamped])

	rospy.spin()

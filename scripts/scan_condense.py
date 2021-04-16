#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class scanCondense:
	def __init__(self):
		rospy.init_node('scan_condense')
		rospy.Subscriber('scan', LaserScan, self.scan_cb)
		self.scan_output_size = 72
		self.fov = 144
		self.DEG2RAD = 0.01745329252
		self.scan_pub = rospy.Publisher('/scan_condensed', LaserScan, queue_size=1000)

	def scan_cb(self, msg):
		condensed_scan = LaserScan()
		condensed_scan.header = msg.header
		condensed_scan.angle_min = msg.angle_min
		condensed_scan.angle_max = msg.angle_max
		condensed_scan.angle_increment = (self.fov/self.scan_output_size) * self.DEG2RAD
		condensed_scan.time_increment = msg.time_increment
		condensed_scan.scan_time = msg.scan_time
		condensed_scan.range_min = msg.range_min
		condensed_scan.range_max = msg.range_max
		condensed_scan.intensities = [0.0] * self.scan_output_size

		patial_scan = []
		for i in range(len(msg.ranges)):
			patial_scan.append(msg.ranges[i])
			if len(patial_scan) == int(len(msg.ranges)/self.scan_output_size):
				condensed_scan.ranges.append(patial_scan[len(patial_scan)//2])
				patial_scan = []
			if len(condensed_scan.ranges) == self.scan_output_size: break

		self.scan_pub.publish(condensed_scan)
				

if __name__ == '__main__':
	scanCondense()
	rospy.spin()
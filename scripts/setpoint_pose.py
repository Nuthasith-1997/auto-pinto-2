#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class setpointPose:
	def __init__(self, look_ahead, sent_pose_radius):
		rospy.init_node('setpoint_pose')

		self.look_ahead = look_ahead
		self.sent_pose_radius = sent_pose_radius

		rospy.Subscriber('/global_planner/plan', Path, self.plan_cb)
		rospy.Subscriber('/mavros/local_position/odom', Odometry, self.current_pose_cb)

		self.pub_plan = rospy.Publisher('/setpoint_plan', Path, queue_size=10)
		self.pub_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

		self.plan = []
		self.setpoint_x = 0
		self.setpoint_y = 0
		self.current_pose_x = 0
		self.current_pose_y = 0
		self.goal_pose = PoseStamped()
		self.goal_pose.pose.position.x = 0
		self.goal_pose.pose.position.y = 0
		self.goal_pose.pose.orientation = (0,0,0,1)

		self.no_plan = True
		self.goal = False

	def plan_cb(self, msg):
		self.plan = msg.poses
		self.no_plan = False
		
		if self.look_ahead > len(self.plan):
			if not self.goal:
				self.goal_pose = msg.poses[-1] # msg_type: PoseStamped
				self.goal = True
		else:
			self.goal = False

	def pose_from_gradient(self):
		self.interested_plan = self.plan[:min(self.look_ahead, len(self.plan))]

		yaw_array = []
		yaw_gradient = [0]
		for i in range(min(self.look_ahead, len(self.plan))):
			yaw_array.append(euler_from_quaternion(interested_plan[i].pose.orientation)[2])
			if i > 0:
				yaw_gradient.append(abs(yaw_array[i]-yaw_array[i-1]))

		max_index = yaw_gradient.index(max(yaw_gradient))

		self.setpoint_x = interested_plan[max_index].pose.position.x
		self.setpoint_y = interested_plan[max_index].pose.position.y
		self.setpoint_q = interested_plan[max_index].pose.orientation

	def current_pose_cb(self, msg):
		self.current_pose_x = msg.pose.pose.position.x
		self.current_pose_y = msg.pose.pose.position.y

	def send_pose(self):
		if (((self.current_pose_x-self.setpoint_x)**2 + (self.current_pose_y-self.setpoint_y)**2)**0.5 < self.sent_pose_radius) and not self.no_plan:
			if ((self.goal_pose.pose.position.x-self.current_pose_x)**2 + (self.goal_pose.pose.position.y-self.current_pose_y)**2)**0.5 > self.sent_pose_radius
				self.pose_from_gradient()

				sent_pose = PoseStamped()
				i = 0
				sent_pose.header.seq = i
				sent_pose.header.stamp = rospy.Time.now()
				sent_pose.header.frame_id = ""
				sent_pose.pose.position.x = self.setpoint_x
				sent_pose.pose.position.y = self.setpoint_y
				sent_pose.pose.position.z = 0
				sent_pose.pose.orientation = self.setpoint_q
				self.pub_pose.publish(sent_pose)

				self.no_plan = True
			
			else:# send goal
				sent_pose = PoseStamped()
				i = 0
				sent_pose.header.seq = i
				sent_pose.header.stamp = rospy.Time.now()
				sent_pose.header.frame_id = ""
				sent_pose.pose.position.x = self.goal_pose.pose.position.x
				sent_pose.pose.position.y = self.goal_pose.pose.position.y
				sent_pose.pose.position.z = 0
				sent_pose.pose.orientation = self.goal_pose.pose.orientation
				self.pub_pose.publish(sent_pose)

				self.no_plan = True

if __name__ == '__main__':
	rate = rospy.Rate(1)
	sp = setpointPose(10, 0.2)

	while not rospy.is_shutdown():
		sp.send_pose()
		rate.sleep()
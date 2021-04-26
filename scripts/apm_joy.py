#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode

class joyStick:
	def __init__(self):
		rospy.init_node('apm_joy')
		print "Node 'apm_joy' has initialized."

		rospy.Subscriber('/joy', Joy, self.joyCb)
		self.pub_rc_in = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

		self.rc_in = OverrideRCIn()
		self.rc_in.channels = [1500, 0, 1500, 0, 0, 0, 0, 0]
		# The first element is translational motion along x-axis of vehicle frame.
		# The thrid element is rotation about z-axis of vehicle frame (yaw rotation).

		self.check = False
		self.is_acro = False
		self.is_armed = True

	# ACRO mode enable cruise and steering controller to send servo output while feeding back
	# the translational velocity and turn rate from Ardupilot's EKF and AHRS.
	# This mode is more reliable than MANUAL mode, unless AHRS_HEALTH is bad.
	def setAcro(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			flightModeService(custom_mode='ACRO')
			print "ACRO mode is being set..."
		except rospy.ServiceException as e:
			print "Service set_mode call faild: {}. Acro mode could not be set.".format(e)

	# MANUAL is Ardurover default mode when start ardupilot. It's open loop control working 
	# by receive RC input then convert to servo output to the motors.
	def setManual(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			flightModeService(custom_mode='MANUAL')
			print "MANUAL mode is being set..."
		except rospy.ServiceException as e:
			print "Service set_mode call faild: {}. Manual mode could not be set.".format(e)

	# When Ardupilot is disarmed, all wheel motor commands are blocked.
	def setArm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			armService(True)
			print "Motors are being armed..."
		except rospy.ServiceException as e:
			print "Service arming call faild: {}".format(e)

	def setDisarm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			armService(False)
			print "Motor are being disarmed..."
		except rospy.ServiceException as e:
			print "Service disarming call faild: {}".format(e)

	def joyCb(self, msg):
		# User have to hold the 4th button in order to send moving cmd.
		if msg.buttons[4] == 1:
			self.check = True
		else: self.check = False

		# Arm/Disarm toggle.
		if msg.buttons[7] == 1:
			if self.is_armed:
				self.setDisarm()
				self.is_armed = False			
			else:
				self.setArm()
				self.is_armed = True

		# Acro/Manual mode toggle.
		if msg.buttons[6] == 1:
			if self.is_acro:
				self.setManual()
				self.is_acro = False
			else:
				self.setAcro()
				self.is_acro = True
			
		# PWM = 1500 is middle point of RC input.	
		# Max PWM = 2000 (max forward motion)
		# Min PWM = 1000 (max reverse motion)
		self.rc_in.channels[0] = 1500 + msg.axes[1]*500
		self.rc_in.channels[2] = 1500 + msg.axes[2]*500

	def pubRCIn(self):
		if not self.check:
			# If button 4 isn't being hold, send zero motion RC input instead.
			#self.rc_in.channels = [1500, 0, 1500, 0, 0, 0, 0, 0]
			self.rc_in.channels[0] = 1500
			self.rc_in.channels[2] = 1500

		self.pub_rc_in.publish(self.rc_in)

if __name__ == '__main__':
	js = joyStick()
	rate = rospy.Rate(30)
	# 30 Hz

	while not rospy.is_shutdown():
		js.pubRCIn()
		rate.sleep()
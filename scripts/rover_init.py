#!/usr/bin/env python

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import GeoPointStamped


class fcuModes:
	def __init__(self):
		self.state = State()
		rospy.Subscriber('/mavros/state', State, self.stateCb)

	def setEKFOrigin(self):
		gp_origin_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=1, latch = True)

		global_origin = GeoPointStamped()
		global_origin.header.stamp = rospy.Time.now()
		global_origin.header.seq = 0
		global_origin.header.frame_id = ''
		global_origin.position.latitude = 13.736444
		global_origin.position.longitude = 100.533986
		global_origin.position.altitude = 0

		gp_origin_pub.publish(global_origin)

	def setSR(self, sr):
		"""
		rospy.wait_for_service('/mavros/param/set')
		try:
			srParamSet = rospy.ServiceProxy('/mavros/param/set', mavros_msgs.srv.ParamSet)
			valueSet = ParamValue()
			valueSet.real = sr
			#srParamSet(param_id ="SR0_EXTRA1", value = valueSet)
			srParamSet(param_id ="SR0_POSITION", value = valueSet)
			srParamSet(param_id ="SR1_POSITION", value = valueSet)
			srParamSet(param_id ="SR2_POSITION", value = valueSet)
			srParamSet(param_id ="SR3_POSITION", value = valueSet)
		except rospy.ServiceException as e:
			print("Service set sr param call faild: {}".format(e))
		"""
		rospy.wait_for_service('/mavros/set_stream_rate')
		try:
			srService = rospy.ServiceProxy('/mavros/set_stream_rate', mavros_msgs.srv.StreamRate)
			# http://docs.ros.org/en/hydro/api/mavros/html/srv/StreamRate.html
			# https://github.com/ArduPilot/ardupilot/issues/10881
			stream_id = 6
			message_rate = sr
			on_off = True
			srService(stream_id, message_rate, on_off)
		except rospy.ServiceException as e:
			print("Service set stream rate call faild: {}".format(e))

	def stateCb(self, msg):
		self.state = msg
	
	def setArm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException as e:
			print("Service arming call faild: {}".format(e))

	def setDisarm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(False)
		except rospy.ServiceException as e:
			print("Service disarming call faild: {}".format(e))

	def setManualMode(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='MANUAL')
		except rospy.ServiceException as e:
			print("Service set_mode call faild: {}. Manual mode could not be set.".format(e))

	def setGuidedMode(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='GUIDED')
		except rospy.ServiceException as e:
			print("Service set_mode call faild: {}. Guided mode could not be set.".format(e))

	def setHome(self):
		rospy.wait_for_service('/mavros/cmd/set_home')
		try:
			setHomeService = rospy.ServiceProxy('/mavros/cmd/set_home', mavros_msgs.srv.CommandHome)
			current_gps = False
			yaw = 0.0
			latitude = 13.736444
			longtitude = 100.533986
			altitude = 0.0
			setHomeService(current_gps, yaw, latitude, longtitude, altitude)
		except rospy.ServiceException as e:
			print("Service set_home call faild: {}. Home could not be set.".format(e))

	def setExtNavMode(self):
		rospy.wait_for_service('/mavros/param/set')
		try:
			extNavService = rospy.ServiceProxy('/mavros/param/set', mavros_msgs.srv.ParamSet)
			
			# the default to use EKF2
			valueSet = ParamValue()
			valueSet.integer = 2
			valueSet.real = 0.0
			extNavService(param_id ="AHRS_EKF_TYPE", value = valueSet)
			# the default to enable EK2
			valueSet.integer = 1
			extNavService(param_id = "EK2_ENABLE", value = valueSet)
			# the default to disable EK3
			valueSet.integer = 0
			extNavService(param_id = "EK3_ENABLE", value = valueSet)
			# to disable the GPS
			valueSet.integer = 0
			extNavService(param_id = "GPS_TYPE", value = valueSet)
			# to disable the EKF use of the GPS
			valueSet.integer = 3
			extNavService(param_id = "EK2_GPS_TYPE", value = valueSet)
			# unknown
			valueSet.integer = 0
			valueSet.real = 0.1
			extNavService(param_id = "EK2_POSNE_M_NSE", value = valueSet)
			# unknown
			extNavService(param_id = "EK2_VELD_M_NSE", value = valueSet)
			# unknown
			extNavService(param_id = "EK2_VELNE_M_NSE", value = valueSet)
			#to disable the EKF use of the compass and instead rely on the heading from external navigation data.
			valueSet.integer = 0
			valueSet.real = 0.0
			#extNavService(param_id = "MAG_ENABLE", value = valueSet)
			extNavService(param_id = "COMPASS_USE", value = valueSet)
			extNavService(param_id = "COMPASS_USE2", value = valueSet)
			extNavService(param_id = "COMPASS_USE3", value = valueSet)

			#setup FCU param SYSID_MYGCS to match mavros system id (to accept control from mavros)
			valueSet.integer = 1
			extNavService(param_id = "SYSID_MYGCS", value = valueSet)

		except rospy.ServiceException as e:
			print("Service set_param call failed: {}. External navigation parameters could not be set.".format(e))

	def setDefaultNavMode(self):
		rospy.wait_for_service('/mavros/param/set')
		try:
			defaultNavService = rospy.ServiceProxy('/mavros/param/set', mavros_msgs.srv.ParamSet)
			
			# the default to use EKF2
			valueSet = ParamValue()
			valueSet.integer = 2
			valueSet.real = 0.0
			defaultNavService(param_id ="AHRS_EKF_TYPE", value = valueSet)
			# the default to enable EK2
			valueSet.integer = 1
			defaultNavService(param_id = "EK2_ENABLE", value = valueSet)
			# the default to disable EK3
			valueSet.integer = 0
			defaultNavService(param_id = "EK3_ENABLE", value = valueSet)
			# to enable the GPS (default)
			valueSet.integer = 1
			defaultNavService(param_id = "GPS_TYPE", value = valueSet)
			# to disable the EKF use of the GPS
			valueSet.integer = 1
			defaultNavService(param_id = "EK2_GPS_TYPE", value = valueSet)
			# unknown
			valueSet.integer = 0
			valueSet.real = 1
			defaultNavService(param_id = "EK2_POSNE_M_NSE", value = valueSet)
			# unknown
			valueSet.real = 0.7
			defaultNavService(param_id = "EK2_VELD_M_NSE", value = valueSet)
			# unknown
			valueSet.real = 0.5
			defaultNavService(param_id = "EK2_VELNE_M_NSE", value = valueSet)
			#Gyro noise
			valueSet.real = 0.03
			defaultNavService(param_id = "EK2_GYRO_P_NSE", value = valueSet)
			#Accelerometer noise
			valueSet.real = 0.6
			defaultNavService(param_id = "EK2_ACC_P_NSE", value = valueSet)
			#to disable the EKF use of the compass and instead rely on the heading from external navigation data.
			valueSet.integer = 1
			valueSet.real = 0.0
			#defaultNavService(param_id = "MAG_ENABLE", value = valueSet)
			defaultNavService(param_id = "COMPASS_USE", value = valueSet)
			defaultNavService(param_id = "COMPASS_USE2", value = valueSet)
			valueSet.integer = 0
			defaultNavService(param_id = "COMPASS_USE3", value = valueSet)

			#setup FCU param SYSID_MYGCS to match mavros system id (to accept control from mavros)
			valueSet.integer = 1
			defaultNavService(param_id = "SYSID_MYGCS", value = valueSet)

		except rospy.ServiceException as e:
			print("Service set_param call failed: {}. Default navigation parameters could not be set.".format(e))

if __name__ == '__main__':
	rospy.init_node('rover_init')
	print "Node 'rover_init' has initialized."

	rate = rospy.Rate(1)

	rover = fcuModes()
	rover.setSR(100)

	# Set to use external navigation instead of GPS or use GPS, enable only 1 of them
	#rover.setExtNavMode()
	#rate.sleep()
	#print "External navigation parameters are set."
	#rover.setDefaultNavMode

	# Set global position origin & new home for guided mode
	rover.setHome()
	rate.sleep()
	print "Home position is set."
	rover.setEKFOrigin()
	rate.sleep()
	print "Global position origin is set."

	rate.sleep()

	#while not rover.state.guided:
	#	rover.setGuidedMode()
	#	print "Attempt to change flight mode ..."
	#	rate.sleep()
	#print "Guided mode is set successfully."
	current_mode = rover.state.mode

	#while not rover.state.armed:
	#	rover.setArm()
	#	print "Attempt to arm the vehicle ..."
	#	rate.sleep()
	#print "The vehicle is arming ..."

	while not rospy.is_shutdown():
		rover.setSR(100)
		if rover.state.mode != current_mode:
			mode = rover.state.mode
			print "The vehicle is now in {} mode.".format(mode)
			current_mode = mode

		rate.sleep()

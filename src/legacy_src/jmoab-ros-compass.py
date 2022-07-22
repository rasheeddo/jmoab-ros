#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int8, Bool
from geometry_msgs.msg import QuaternionStamped
import time
import struct
import numpy as np
import os
import argparse

class JMOAB_COMPASS(object):

	def __init__(self, NS):

		rospy.init_node('jmoab_ros_compass_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-COMPASS node")

		self.bus = SMBus(1)

		## BNO055 address and registers
		self.IMU_ADDR = 0x28

		self.OPR_REG = 0x3d
		self.AXIS_MAP_CONFIG_REG = 0x41
		self.AXIS_MAP_SIGN_REG = 0x42


		self.EUL_X_LSB = 0x1a
		self.EUL_X_MSB = 0x1b
		self.EUL_Y_LSB = 0x1c
		self.EUL_Y_MSB = 0x1d
		self.EUL_Z_LSB = 0x1e
		self.EUL_Z_MSB = 0x1f

		self.QUA_w_LSB = 0x20
		self.QUA_w_MSB = 0x21
		self.QUA_x_LSB = 0x22
		self.QUA_x_MSB = 0x23
		self.QUA_y_LSB = 0x24
		self.QUA_y_MSB = 0x25
		self.QUA_z_LSB = 0x26
		self.QUA_z_MSB = 0x27

		self.ACC_OFF_X_LSB = 0x55
		self.ACC_OFF_X_MSB = 0x56
		self.ACC_OFF_Y_LSB = 0x57
		self.ACC_OFF_Y_MSB = 0x58
		self.ACC_OFF_Z_LSB = 0x59
		self.ACC_OFF_Z_MSB = 0x5a

		self.MAG_OFF_X_LSB = 0x5b
		self.MAG_OFF_X_MSB = 0x5c
		self.MAG_OFF_Y_LSB = 0x5d
		self.MAG_OFF_Y_MSB = 0x5e
		self.MAG_OFF_Z_LSB = 0x5f
		self.MAG_OFF_Z_MSB = 0x60

		self.GYR_OFF_X_LSB = 0x61
		self.GYR_OFF_X_MSB = 0x62
		self.GYR_OFF_Y_LSB = 0x63
		self.GYR_OFF_Y_MSB = 0x64
		self.GYR_OFF_Z_LSB = 0x65
		self.GYR_OFF_Z_MSB = 0x66

		self.ACC_RAD_LSB = 0x67
		self.ACC_RAD_MSB = 0x68

		self.MAG_RAD_LSB = 0x69
		self.MAG_RAD_MSB = 0x6a


		# OPR_REG mode
		self.CONFIG_MODE = 0x00
		self.IMU_MODE = 0x08
		self.NDOF_FMC_OFF = 0x0b
		self.NDOF_MODE = 0x0C

		# AXIS REMAP
		self.REMAP_DEFAULT = 0x24
		self.REMAP_X_Y = 0x21
		self.REMAP_Y_Z = 0x18
		self.REMAP_Z_X = 0x06
		self.REMAP_X_Y_Z_TYPE0 = 0x12
		self.REMAP_X_Y_Z_TYPE1 = 0x09

		# AXIS SIG
		self.SIGN_DEFAULT = 0x00	# heatsink is down, usb backward
		self.Z_REV = 0x01			# heatsink is up, usb backward
		self.YZ_REV = 0x03			# heatsink is up, usb forward

		self.pre_calib = []
		rospy.loginfo(("Read calibration_offset.txt file"))
		jmoab_calib_dir = "/home/nvidia/catkin_ws/src/jmoab-ros/example"
		calib_file_name = "calibration_offset.txt"
		calib_file_dir = os.path.join(jmoab_calib_dir, calib_file_name)
		if os.path.exists(calib_file_dir):
			with open(calib_file_dir, "r") as f:
				for line in f:
					self.pre_calib.append(int(line.strip()))
		else:
			print("There is no calibration_offset.txt file!")
			print("Do the calibration step first")
			quit()

		self.imu_int()

		self.checking_non_zero_start()

		####################
		## heading offset ##
		####################
		self.hdg_offset = 0.0
		rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))
		self.lat = 0.0
		self.lon = 0.0
		# self.calib_flag = False
		self.get_latlon_once = True
		self.ch7_from_high = False
		self.start_hdg = 0.0

		# self.sbus_steering = 969
		# self.sbus_throttle = 1160

		# self.hdg_ref = 0.0
		# self.hdg_ref_flag = False
		# self.hdg_ref_timestamp = time.time()
		# self.hdg_ref_timeout = 10.0

		self.cart_mode = 0

		self.sbus_cmd_throttle = 1024
		self.sbus_cmd_steering = 1024
		self.fix_stat = 0
		self.sbus_steering_stick = 1024
		self.sbus_throttle_stick = 1024
		self.prev_lat = self.lat
		self.prev_lon = self.lon
		self.pure_hdg = 0.0
		self.sign = 1.0
		self.brg = 0.0
		self.last_time_manual_cal = time.time()
		self.last_time_auto_cal = time.time()
		self.cal_offset = False
		self.do_estimation = False

		## Kalman filter of hdg_offset estimate
		self.hdg_off_est = 20.0 # first guess
		self.state_predict = self.hdg_off_est
		self.error_est = 10.0 
		self.error_mea = 8.0 # a variance of

		#############
		## Pub/Sub ##
		#############
		self.compass_pub = rospy.Publisher(self.namespace_attaching(NS, "/jmoab_compass"), Float32MultiArray, queue_size=10)
		self.compass_msg = Float32MultiArray()

		rospy.Subscriber(self.namespace_attaching(NS, "/ublox/fix"), NavSatFix, self.gps_callback)
		rospy.Subscriber(self.namespace_attaching(NS, "/sbus_rc_ch"), Int32MultiArray, self.sbus_callback)
		rospy.Subscriber(self.namespace_attaching(NS, "/atcart_mode"), Int8, self.atcart_mode_callback)
		rospy.Subscriber(self.namespace_attaching(NS, "/sbus_cmd"), Int32MultiArray, self.sbus_cmd_callback)

		rospy.loginfo("Publishing Roll-Pitch-Heading {:} topic respect to true north".format(self.namespace_attaching(NS, "/jmoab_compass")))
		# rospy.loginfo("Publishing Calibration flag as /hdg_calib_flag in case of calibrating")

		self.loop()
		rospy.spin()

	def namespace_attaching(self, NS, topic_name):
		if NS is None:
			return topic_name
		else:
			if NS.startswith("/"):
				topic_name = NS + topic_name
			else:
				topic_name = "/" + NS + topic_name
			return topic_name

	def gps_callback(self, msg):

		self.prev_lat = self.lat
		self.prev_lon = self.lon

		self.lat = msg.latitude
		self.lon = msg.longitude

		self.fix_stat = msg.status.status

		###########################################################################
		## we put the calculation/estimation inside the gps_callback             ##
		## because the sampling time of gps is slowest,                          ##
		## so we could get the real difference between current and previous data ##
		###########################################################################
		if self.do_estimation:
			self.brg = self.get_bearing(self.prev_lat, self.prev_lon, self.lat, self.lon)
			self.brg = self.ConvertTo360Range(self.brg)

			if self.cart_mode == 1:

				self.period_manual_cal = time.time() - self.last_time_manual_cal

				if self.period_manual_cal > 1.0:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.brg, self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)
			elif self.cart_mode == 2:

				self.period_auto_cal = time.time() - self.last_time_auto_cal

				if self.period_auto_cal > 0.3:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.brg, self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)


	def sbus_cmd_callback(self, msg):

		self.sbus_cmd_steering = msg.data[0]
		self.sbus_cmd_throttle = msg.data[1]

	# def heading_ref_callback(self, data):
	# 	qw = data.quaternion.w
	# 	qx = data.quaternion.x
	# 	qy = data.quaternion.y
	# 	qz = data.quaternion.z

	# 	r11 = qw**2 + qx**2 - qy**2 - qz**2 #1 - 2*qy**2 - 2*qz**2
	# 	r12 = 2*qx*qy - 2*qz*qw
	# 	r13 = 2*qx*qz + 2*qy*qw
	# 	r21 = 2*qx*qy + 2*qz*qw
	# 	r22 = qw**2 - qx**2 + qy**2 - qz**2	#1 - 2*qx**2 - 2*qz**2
	# 	r23 = 2*qy*qz - 2*qx*qw
	# 	r31 = 2*qx*qz - 2*qy*qw
	# 	r32 = 2*qy*qz + 2*qx*qw
	# 	r33 = qw**2 - qx**2 - qy**2 + qz**2	#1 - 2*qx**2 - 2*qy**2
	# 	rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

	# 	self.hdg_ref = self.ConvertTo360Range(np.degrees(np.arctan2(rot[1,0],rot[0,0])))
	# 	self.hdg_ref_flag = True
	# 	self.hdg_ref_timestamp = time.time()

	def atcart_mode_callback(self, msg):
		self.cart_mode = msg.data

	def sbus_callback(self, msg):

		self.sbus_steering_stick = msg.data[0]
		self.sbus_throttle_stick = msg.data[1]

		# if msg.data[6] > 1500:
		# 	self.calib_flag = True
		# else:
		# 	self.calib_flag = False

	def imu_int(self):

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.CONFIG_MODE)
		time.sleep(0.05)	# 7ms for operating mode

		## comment this if no need config
		# self.config_axis_remap()
		self.config_axis_sign(self.SIGN_DEFAULT)

		self.write_pre_calib()

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.NDOF_MODE)
		time.sleep(0.05)	# 7ms for operating mode

		offset_data = self.bus.read_i2c_block_data(self.IMU_ADDR, self.ACC_OFF_X_LSB, 22)
		rospy.loginfo("calibration offset {:s}".format(offset_data))

	def write_pre_calib(self):
		timeSleep = 0.05
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_OFF_X_LSB, self.pre_calib[0])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_OFF_X_MSB, self.pre_calib[1])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_OFF_Y_LSB, self.pre_calib[2])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_OFF_Y_MSB, self.pre_calib[3])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_OFF_Z_LSB, self.pre_calib[4])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_OFF_Z_MSB, self.pre_calib[5])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_OFF_X_LSB, self.pre_calib[6])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_OFF_X_MSB, self.pre_calib[7])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_OFF_Y_LSB, self.pre_calib[8])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_OFF_Y_MSB, self.pre_calib[9])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_OFF_Z_LSB, self.pre_calib[10])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_OFF_Z_MSB, self.pre_calib[11])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.GYR_OFF_X_LSB, self.pre_calib[12])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.GYR_OFF_X_MSB, self.pre_calib[13])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.GYR_OFF_Y_LSB, self.pre_calib[14])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.GYR_OFF_Y_MSB, self.pre_calib[15])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.GYR_OFF_Z_LSB, self.pre_calib[16])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.GYR_OFF_Z_MSB, self.pre_calib[17])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_RAD_LSB, self.pre_calib[18])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.ACC_RAD_MSB, self.pre_calib[19])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_RAD_LSB, self.pre_calib[20])
		time.sleep(timeSleep)
		self.bus.write_byte_data(self.IMU_ADDR, self.MAG_RAD_MSB, self.pre_calib[21])
		time.sleep(timeSleep)

	def config_axis_sign(self, SIGN):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_SIGN_REG, SIGN)
		time.sleep(0.1)	# 19ms from any mode to config mode

	def config_remap(self, REMAP):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_CONFIG_REG, REMAP)
		time.sleep(0.1)	# 19ms from any mode to config mode

	def checking_non_zero_start(self):

		rospy.loginfo("Checking non-zero position from start...")
		restart_tic = time.time()
		while True:
			raw = self.bus.read_i2c_block_data(self.IMU_ADDR, self.EUL_X_LSB, 6)
			hdg,roll,pitch = struct.unpack('<hhh', bytearray(raw))

			hdg = hdg/16.0
			roll = roll/16.0
			pitch = pitch/16.0

			# print(hdg)

			if (hdg == 0.0) or (hdg == 360.0):
				rospy.loginfo("heading is 0.0, try running the code again")

				## Give a chance to escape from  0.0 in 5 seconds, then quit restart the code again
				restart_timeout = time.time() - restart_tic
				if restart_timeout > 5.0:
					quit()
			else:
				rospy.loginfo("Compass is ready")
				break

			time.sleep(0.05)

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg

	def ConvertTo180Range(self, deg):

		deg = self.ConvertTo360Range(deg)
		if deg > 180.0:
			deg = -(180.0 - (deg%180.0))

		return deg

	def get_bearing(self, lat1, lon1, lat2, lon2):

		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		y = np.sin(dLon)*np.cos(lat_end)
		x = np.cos(lat_start)*np.sin(lat_end) - np.sin(lat_start)*np.cos(lat_end)*np.cos(dLon)
		bearing = np.degrees(np.arctan2(y,x))

		return bearing


	def find_smallest_diff_ang(self, goal, cur):

		## goal is in 180ranges, we need to convert to 360ranges first

		diff_ang1 = abs(self.ConvertTo360Range(goal) - cur)

		if diff_ang1 > 180.0:

			diff_ang = 180.0 - (diff_ang1%180.0)
		else:
			diff_ang = diff_ang1

		## check closet direction
		compare1 = self.ConvertTo360Range(self.ConvertTo360Range(goal) - self.ConvertTo360Range(cur + diff_ang))
		compare2 = self.ConvertTo180Range(goal - self.ConvertTo180Range(cur + diff_ang))
		# print(compare1, compare2)
		if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
			sign = 1.0 # clockwise count from current hdg to target
		else:
			sign = -1.0 # counter-clockwise count from current hdg to target


		return diff_ang, sign

	def MovingAverage(self, data_raw, data_array, array_size):

		if len(data_array) < array_size:
			data_array = np.append(data_array, data_raw)
		else:
			data_array = np.delete(data_array, 0)
			data_array = np.append(data_array, data_raw)
		data_ave = np.average(data_array)
		return data_ave, data_array

	def kalman_filter(self, measure, prev_state_est, prev_error_est):

		## reference
		## https://www.kalmanfilter.net/kalman1d.html

		######################
		## state estimation ##
		######################
		KG = prev_error_est/(prev_error_est + self.error_mea)
		cur_state_est = prev_state_est + KG*(measure - prev_state_est)
		cur_error_est = (1 - KG)*(prev_error_est)

		################
		## prediction ##
		################
		self.error_est = cur_error_est + 0.01 # 0.01 is process noise, could help when hdg_offset is fluctuating during time
		
		# hdg_offset is not dynamic behaviour, so predicted state is constant as estimated state
		self.state_predict = cur_state_est  

		return cur_state_est
	
	def loop(self):
		rate = rospy.Rate(100) # 10hz
		# pure_hdg = 0.0

		while not rospy.is_shutdown():
			startTime = time.time()

			raw = self.bus.read_i2c_block_data(self.IMU_ADDR, self.EUL_X_LSB, 6)
			pure_hdg,roll,pitch = struct.unpack('<hhh', bytearray(raw))

			pure_hdg = pure_hdg/16.0
			roll = roll/16.0
			pitch = pitch/16.0
			# pure_hdg = self.ConvertTo360Range(hdg)
			self.pure_hdg = self.ConvertTo360Range(pure_hdg)
			
			# hdg = self.ConvertTo360Range(hdg - self.hdg_offset)
			hdg = self.ConvertTo360Range(self.pure_hdg + (self.sign)*self.hdg_off_est)

			#############################################################
			## hdg_offset calculation by two points gps bearing angle, ##
			## and estimation by Kalman Filter                         ##
			#############################################################

			## must be rtk-fixed, to get precise position
			if self.fix_stat == 2:
				## in manual case
				if self.cart_mode == 1:
					###############################################################
					## we do estimation only when throttle is up and no steering ##
					###############################################################
					if (self.sbus_throttle_stick > 1048) and (924 < self.sbus_steering_stick < 1124):
						
						self.do_estimation = True

					else:
						self.last_time_manual_cal = time.time()
						self.last_time_auto_cal = time.time()
						self.cal_offset = False
						self.do_estimation = False

				## in auto case
				elif self.cart_mode == 2:
					#######################################################
					## we do estimation only when sbus throttle is high, ##
					## and sbus steering is no curvy                     ##
					#######################################################
					if (self.sbus_cmd_throttle > 1090) and (970 < self.sbus_cmd_steering < 1074):

						self.do_estimation = True

					else:
						self.last_time_manual_cal = time.time()
						self.last_time_auto_cal = time.time()
						self.cal_offset = False
						self.do_estimation = False

				else:
					self.last_time_manual_cal = time.time()
					self.last_time_auto_cal = time.time()

			print("p_hdg: {:.2f} | hdg: {:.2f} | hdg_off: {:.2f} | hdg_off_est: {:.2f} | brg: {:.2f} | cal_offset: {:}".format(\
				self.pure_hdg, hdg, self.hdg_offset, self.hdg_off_est, self.brg, self.cal_offset))


			#######################################
			### Live heading offset calibration ###
			#######################################
			# if self.calib_flag:
			# 	if self.get_latlon_once:
			# 		lat_start = self.lat
			# 		lon_start = self.lon
			# 		self.get_latlon_once = False
			# 		self.hdg_offset = 0.0
			# 		# hdg = self.ConvertTo360Range(hdg-self.hdg_offset)
			# 		self.start_hdg = pure_hdg
			# 		rospy.loginfo("Start heading offset calibrating with start_hdg {:.2f} hdg_offset {:.2f}".format(self.start_hdg, self.hdg_offset))

			# 		######################################################################
			# 		## Set hdg_calib_flag to True to let other autopilot node knows     ##
			# 		## Set atcart_mode to auto, we're gonna use auto mode straight line ##
			# 		######################################################################
			# 		self.hdg_calib_flag_msg.data = True
			# 		self.hdg_calib_flag_pub.publish(self.hdg_calib_flag_msg)
			# 		self.atcart_mode_cmd_msg.data = 2
			# 		self.atcart_mode_cmd_pub.publish(self.atcart_mode_cmd_msg)
			# 		time.sleep(0.5)	# a bit of delay to make sure it's not gonna immediately move

			# 	###############################
			# 	## keep getting last lat/lon ##
			# 	###############################
			# 	lat_last = self.lat
			# 	lon_last = self.lon

			# 	self.ch7_from_high = True
			# 	#################################
			# 	## keep drive as straight line ##
			# 	#################################
			# 	self.sbus_cmd.data = [self.sbus_steering, self.sbus_throttle]
			# 	self.sbus_cmd_pub.publish(self.sbus_cmd)



			# elif (self.calib_flag == False) and self.ch7_from_high:
			# 	##################################################################
			# 	## switch atcart_mode back to manual to make it completely stop ##
			# 	## send False on hdg_calib_flag to let autopilot node knows     ##
			# 	##################################################################
			# 	self.atcart_mode_cmd_msg.data = 1
			# 	self.atcart_mode_cmd_pub.publish(self.atcart_mode_cmd_msg)
			# 	time.sleep(0.5) # a bit of delay makes other autopilot node not freak out
			# 	self.hdg_calib_flag_msg.data = False
			# 	self.hdg_calib_flag_pub.publish(self.hdg_calib_flag_msg)

			# 	self.sbus_cmd.data = [1024, 1024]
			# 	self.sbus_cmd_pub.publish(self.sbus_cmd)


			# 	self.ch7_from_high = False
			# 	line_hdg = self.get_bearing(lat_start, lon_start, lat_last, lon_last)
			# 	self.hdg_offset = self.start_hdg - self.ConvertTo360Range(line_hdg)
			# 	rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))


			# else:
			# 	self.get_latlon_once = True
			# 	self.ch7_from_high = False


			self.compass_msg.data = [roll, pitch, hdg]
			self.compass_pub.publish(self.compass_msg)

			period = time.time() - startTime 
			# print("period", period)
			if period < 0.007:
				sleepMore = 0.007 - period
				time.sleep(sleepMore)

			rate.sleep()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Compass node of jmoab-ros')
	parser.add_argument('--ns',
						help="a namespace in front of original topic")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	ns = args.ns

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")
		
	jmoab_compass = JMOAB_COMPASS(ns)
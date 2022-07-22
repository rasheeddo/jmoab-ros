#! /usr/bin/env python
import rospy
import rospkg
from smbus2 import SMBus
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, Int16MultiArray, UInt8, Bool
from geometry_msgs.msg import QuaternionStamped, Twist
import time
import struct
import numpy as np
import os
import argparse

## BNO055 address
IMU_ADDR = 0x28

## Registers and Values
### config
OPR_REG = 0x3d
SYS_TRIGGER = 0x3F
AXIS_MAP_CONFIG_REG = 0x41
AXIS_MAP_SIGN_REG = 0x42
### OPR_REG mode
CONFIG_MODE = 0x00
IMU_MODE = 0x08
NDOF_FMC_OFF = 0x0b
NDOF_MODE = 0x0C
### Reset ###
RST_SYS = 0x20 # 5th bit
### AXIS REMAP
REMAP_DEFAULT = 0x24
REMAP_X_Y = 0x21
REMAP_Y_Z = 0x18
REMAP_Z_X = 0x06
REMAP_X_Y_Z_TYPE0 = 0x12
REMAP_X_Y_Z_TYPE1 = 0x09
### AXIS SIG
SIGN_DEFAULT = 0x00	
Z_REV = 0x01			
YZ_REV = 0x03	
### data
GYRO_X_LSB = 0x14
EUL_Heading_LSB = 0x1a
QUA_W_LSB = 0x20
ACC_X_LSB = 0x28
### offset 
ACC_OFF_X_LSB = 0x55
ACC_OFF_X_MSB = 0x56
ACC_OFF_Y_LSB = 0x57
ACC_OFF_Y_MSB = 0x58
ACC_OFF_Z_LSB = 0x59
ACC_OFF_Z_MSB = 0x5a
MAG_OFF_X_LSB = 0x5b
MAG_OFF_X_MSB = 0x5c
MAG_OFF_Y_LSB = 0x5d
MAG_OFF_Y_MSB = 0x5e
MAG_OFF_Z_LSB = 0x5f
MAG_OFF_Z_MSB = 0x60
GYR_OFF_X_LSB = 0x61
GYR_OFF_X_MSB = 0x62
GYR_OFF_Y_LSB = 0x63
GYR_OFF_Y_MSB = 0x64
GYR_OFF_Z_LSB = 0x65
GYR_OFF_Z_MSB = 0x66
ACC_RAD_LSB = 0x67
ACC_RAD_MSB = 0x68
MAG_RAD_LSB = 0x69
MAG_RAD_MSB = 0x6a

class JMOAB_COMPASS_2GPS(object):

	def __init__(self, NS):

		rospy.init_node('jmoab_ros_compass_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-COMPASS node")

		self.bus = SMBus(1)

		self.pre_calib = []
		rospy.loginfo(("Read calibration_offset.txt file"))
		rospack = rospkg.RosPack()
		jmoab_ros_path = rospack.get_path("jmoab-ros")
		calib_file_name = "calibration_offset.txt"
		calib_file_dir = os.path.join(jmoab_ros_path, "example", calib_file_name)
		if os.path.exists(calib_file_dir):
			with open(calib_file_dir, "r") as f:
				for line in f:
					self.pre_calib.append(int(line.strip()))
		else:
			print("There is no calibration_offset.txt file!")
			print("Do the calibration step first")
			quit()

		self.imu_int()

		# self.checking_non_zero_start()

		####################
		## heading offset ##
		####################
		self.hdg_offset = 0.0
		rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))
		self.lat = 0.0
		self.lon = 0.0

		self.sbus_steering = 969
		self.sbus_throttle = 1160

		self.cart_mode = 0
		self.vx = 0.0
		self.wz = 0.0

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
		self.hdg_off_est = 0.0 # first guess
		self.state_predict = self.hdg_off_est
		self.error_est = 10.0 
		self.error_mea = 8.0 # a variance of

		## 2nd GPS
		self.lat2 = 0
		self.lon2 = 0
		self.fix_stat2 = 0
		#self.calculate_freq = 5.0 # Hz
		self.true_hdg_from_2gps = 0.0
		self.last_time_calculate = time.time()

		#############
		## Pub/Sub ##
		#############
		ahrs_topic = self.namespace_attaching(NS, "/jmoab/ahrs")
		imu_topic = self.namespace_attaching(NS, "/imu/data")
		gps_topic = self.namespace_attaching(NS, "/ublox/fix")
		gps2_topic = self.namespace_attaching(NS, "/ublox2/fix")
		sbus_rc_topic = self.namespace_attaching(NS, "/jmoab/sbus_rc_ch")
		cart_mode_topic = self.namespace_attaching(NS, "/jmoab/cart_mode")
		cmd_vel_topic = self.namespace_attaching(NS, "/cmd_vel")
		self.ahrs_pub = rospy.Publisher(ahrs_topic, Float32MultiArray, queue_size=1)
		self.ahrs_msg = Float32MultiArray()
		self.imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=1)
		self.imu_msg = Imu()

		rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
		rospy.Subscriber(gps2_topic, NavSatFix, self.gps2_callback)
		rospy.Subscriber(sbus_rc_topic, Int16MultiArray, self.sbus_rc_callback)
		rospy.Subscriber(cart_mode_topic, UInt8, self.cart_mode_callback)
		rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

		rospy.loginfo("Namespace is {:}".format(NS))
		rospy.loginfo("Publishing to {:} topic as std_msgs/Float32MultiArray ex: [roll, pitch, hdg] in degrees".format(ahrs_topic))
		rospy.loginfo("Publishing to {:} topic as sensor_msgs/Imu ex: as quaternion".format(imu_topic))

		rospy.loginfo("Subscribing on {:} topic as sensor_msgs/NavSatFix for true north heading calibration".format(gps_topic))
		rospy.loginfo("Subscribing on {:} topic as sensor_msgs/NavSatFix for true north heading calibration".format(gps2_topic))
		rospy.loginfo("Subscribing on {:} topic as std_msgs/Int16MultiArray for true north heading calibration".format(sbus_rc_topic))
		rospy.loginfo("Subscribing on {:} topic as std_msgs/UInt8 for true north heading calibration".format(cart_mode_topic))
		rospy.loginfo("Subscribing on {:} topic as geometry_msgs/Twist for true north heading calibration".format(cmd_vel_topic))

		self.loop()
		rospy.spin()

	#######################################	
	############ ROS callbacks ############
	#######################################	
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
			# self.brg = self.get_bearing(self.prev_lat, self.prev_lon, self.lat, self.lon)
			# self.brg = self.ConvertTo360Range(self.brg)

			if self.cart_mode == 1:

				self.period_manual_cal = time.time() - self.last_time_manual_cal

				if self.period_manual_cal > 1.0:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.true_hdg_from_2gps , self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)
			elif self.cart_mode == 2:

				self.period_auto_cal = time.time() - self.last_time_auto_cal

				if self.period_auto_cal > 0.3:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.true_hdg_from_2gps, self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)

	def gps2_callback(self, msg):

		self.lat2 = msg.latitude
		self.lon2 = msg.longitude
		self.fix_stat2 = msg.status.status

	def cmd_vel_callback(self, msg):

		self.vx = msg.linear.x
		self.wz = msg.angular.z

	def cart_mode_callback(self, msg):
		self.cart_mode = msg.data

	def sbus_rc_callback(self, msg):

		self.sbus_steering_stick = msg.data[0]
		self.sbus_throttle_stick = msg.data[1]

	############################################	
	######## BNO055 SMBus2 read/write ##########
	############################################
	def imu_int(self):

		## put in reset, to reset all data in registers
		self.bus.write_byte_data(IMU_ADDR, SYS_TRIGGER, RST_SYS)
		time.sleep(1.0)	 ## give it sometime to reset/restart again

		## change to operating mode
		self.bus.write_byte_data(IMU_ADDR, OPR_REG, CONFIG_MODE)
		time.sleep(0.05)	# 7ms for operating mode

		## comment this if no need config
		# self.config_axis_remap()
		self.config_axis_sign(SIGN_DEFAULT)

		self.write_pre_calib()

		## change to operating mode
		self.bus.write_byte_data(IMU_ADDR, OPR_REG, NDOF_MODE)
		time.sleep(0.05)	# 7ms for operating mode


	def write_pre_calib(self):
		timeSleep = 0.05
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_X_LSB, self.pre_calib[0])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_X_MSB, self.pre_calib[1])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Y_LSB, self.pre_calib[2])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Y_MSB, self.pre_calib[3])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Z_LSB, self.pre_calib[4])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_OFF_Z_MSB, self.pre_calib[5])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_X_LSB, self.pre_calib[6])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_X_MSB, self.pre_calib[7])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Y_LSB, self.pre_calib[8])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Y_MSB, self.pre_calib[9])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Z_LSB, self.pre_calib[10])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_OFF_Z_MSB, self.pre_calib[11])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_X_LSB, self.pre_calib[12])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_X_MSB, self.pre_calib[13])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Y_LSB, self.pre_calib[14])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Y_MSB, self.pre_calib[15])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Z_LSB, self.pre_calib[16])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, GYR_OFF_Z_MSB, self.pre_calib[17])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_RAD_LSB, self.pre_calib[18])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, ACC_RAD_MSB, self.pre_calib[19])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_RAD_LSB, self.pre_calib[20])
		time.sleep(timeSleep)
		self.bus.write_byte_data(IMU_ADDR, MAG_RAD_MSB, self.pre_calib[21])
		time.sleep(timeSleep)

	def config_axis_sign(self, SIGN):
		self.bus.write_byte_data(IMU_ADDR, AXIS_MAP_SIGN_REG, SIGN)
		time.sleep(0.1)	# 19ms from any mode to config mode

	def config_remap(self, REMAP):
		self.bus.write_byte_data(IMU_ADDR, AXIS_MAP_CONFIG_REG, REMAP)
		time.sleep(0.1)	# 19ms from any mode to config mode

	def checking_non_zero_start(self):

		rospy.loginfo("Checking non-zero position from start...")
		restart_tic = time.time()
		while True:
			raw = self.bus.read_i2c_block_data(IMU_ADDR, EUL_X_LSB, 6)
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

	def read_euler_angles(self):
		raw = self.bus.read_i2c_block_data(IMU_ADDR, EUL_Heading_LSB, 6)
		yaw,roll,pitch = struct.unpack('<hhh', bytearray(raw))
		yaw = yaw/16.0
		roll = roll/16.0
		pitch = pitch/16.0

		return [roll, pitch, yaw]

	def read_quaternions(self):
		raw_quat = self.bus.read_i2c_block_data(IMU_ADDR, QUA_W_LSB, 8)
		qw,qx,qy,qz = struct.unpack('<hhhh', bytearray(raw_quat))
		qw = qw/16384.0
		qx = qx/16384.0
		qy = qy/16384.0
		qz = qz/16384.0

		return qx,qy,qz,qw

	def raed_gyro(self):
		raw_gyro = self.bus.read_i2c_block_data(IMU_ADDR, GYRO_X_LSB, 6)
		gx,gy,gz = struct.unpack('<hhh', bytearray(raw_gyro))
		gx = np.radians(gx/16.0)
		gy = np.radians(gy/16.0)
		gz = np.radians(gz/16.0)

		return gx,gy,gz

	def read_accel(self):
		raw_acc = self.bus.read_i2c_block_data(IMU_ADDR, ACC_X_LSB, 6)
		ax,ay,az = struct.unpack('<hhh', bytearray(raw_acc))
		ax = ax/100.0
		ay = ay/100.0
		az = az/100.0

		return ax,ay,az

	################################	
	############ Maths #############
	################################

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
		self.error_est = cur_error_est + 0.1 # 0.01 is process noise, could help when hdg_offset is fluctuating during time
		# using 0.1 because we want to trust more on the measuring data which means the hdg_offset from two gps calculation.
		
		# hdg_offset is not dynamic behaviour, so predicted state is constant as estimated state
		self.state_predict = cur_state_est  

		return cur_state_est
	
	def loop(self):
		rate = rospy.Rate(100) # 10hz
		last_log_stamp = time.time()

		while not rospy.is_shutdown():
			startTime = time.time()

			#############################
			### Read data from BNO055 ###
			#############################
			ahrs = self.read_euler_angles()
			quat = self.read_quaternions()
			gyro = self.raed_gyro()
			accl = self.read_accel()

			##########################################
			### Kalman filtering on yaw or heading ###
			##########################################
			self.pure_hdg = self.ConvertTo360Range(ahrs[2])
			
			# hdg = self.ConvertTo360Range(hdg - self.hdg_offset)
			hdg = self.ConvertTo360Range(self.pure_hdg + (self.sign)*self.hdg_off_est)

			#############################################################
			## hdg_offset calculation by two points gps bearing angle, ##
			## and estimation by Kalman Filter                         ##
			#############################################################

			self.true_hdg_from_2gps = self.ConvertTo360Range(self.get_bearing(self.lat, self.lon, self.lat2, self.lon2))

			## must be rtk-fixed on both gps, to get precise position
			if (self.fix_stat == 2) and (self.fix_stat2 == 2):
				## in manual case
				if (self.cart_mode == 1) or (self.cart_mode == 0):
					###############################################################
					## we do estimation only when throttle is up and no steering ##
					###############################################################
					if (1048 < self.sbus_throttle_stick) and (924 < self.sbus_steering_stick < 1124):
						
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
					# if (1000 < self.sbus_cmd_throttle) and (970 < self.sbus_cmd_steering < 1074):

					# 	self.do_estimation = True

					# else:
					# 	self.last_time_manual_cal = time.time()
					# 	self.last_time_auto_cal = time.time()
					# 	self.cal_offset = False
					# 	self.do_estimation = False
					pass


				else:
					self.last_time_manual_cal = time.time()
					self.last_time_auto_cal = time.time()

			if (time.time() - last_log_stamp) > 1.0:
				print("pure_hdg: {:.2f} | true_hdg: {:.2f} | hdg_off: {:.2f} | hdg_off_est: {:.2f} | hdg: {:.2f} | cal_off: {:}".format(\
					self.pure_hdg, self.true_hdg_from_2gps, self.hdg_offset, self.hdg_off_est, hdg, self.cal_offset))
				last_log_stamp = time.time()


			##################################
			### Contruct imu_msg/ ahrs_msg ###
			##################################
			self.imu_msg.header.stamp = rospy.Time.now()
			self.imu_msg.header.frame_id = "imu_link"
			self.imu_msg.orientation.x = quat[0] #roll
			self.imu_msg.orientation.y = quat[1] #pitch
			self.imu_msg.orientation.z = quat[2] #yaw
			self.imu_msg.orientation.w = quat[3] 
			self.imu_msg.orientation_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

			self.imu_msg.angular_velocity.x = gyro[0]
			self.imu_msg.angular_velocity.y = gyro[1]
			self.imu_msg.angular_velocity.z = gyro[2]
			self.imu_msg.angular_velocity_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

			self.imu_msg.linear_acceleration.x = accl[0]
			self.imu_msg.linear_acceleration.y = accl[1]
			self.imu_msg.linear_acceleration.z = accl[2]
			self.imu_msg.linear_acceleration_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

			self.imu_pub.publish(self.imu_msg)

			self.ahrs_msg.data = [ahrs[0], ahrs[1], hdg]
			self.ahrs_pub.publish(self.ahrs_msg)

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
		

	jmoab_compass = JMOAB_COMPASS_2GPS(ns)
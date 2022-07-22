#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import QuaternionStamped
import time
import struct
import numpy as np
import os

class JMOAB_REF_COMPASS:

	def __init__(self):

		rospy.init_node('jmoab_ros_compass_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-Reference-Compass-From-2GPS")

		self.bus = SMBus(1)

		self.compass_pub = rospy.Publisher("/jmoab_compass", Float32MultiArray, queue_size=10)
		self.compass_msg = Float32MultiArray()
		rospy.loginfo("Publishing Roll-Pitch-Heading /jmoab_compass topic respect to true north")

		rospy.Subscriber("/heading", QuaternionStamped, self.heading_ref_callback)

		self.hdg_ref = 0.0

		self.loop()
		rospy.spin()

	def heading_ref_callback(self, data):
		qw = data.quaternion.w
		qx = data.quaternion.x
		qy = data.quaternion.y
		qz = data.quaternion.z

		r11 = qw**2 + qx**2 - qy**2 - qz**2 #1 - 2*qy**2 - 2*qz**2
		r12 = 2*qx*qy - 2*qz*qw
		r13 = 2*qx*qz + 2*qy*qw
		r21 = 2*qx*qy + 2*qz*qw
		r22 = qw**2 - qx**2 + qy**2 - qz**2	#1 - 2*qx**2 - 2*qz**2
		r23 = 2*qy*qz - 2*qx*qw
		r31 = 2*qx*qz - 2*qy*qw
		r32 = 2*qy*qz + 2*qx*qw
		r33 = qw**2 - qx**2 - qy**2 + qz**2	#1 - 2*qx**2 - 2*qy**2
		rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

		self.hdg_ref = np.arctan2(rot[1,0],rot[0,0])
		# pitch = np.arcsin(-rot[2,0])
		# roll = np.arctan2(rot[2,1],rot[2,2])

	def gps_callback(self, msg):

		self.lat = msg.latitude
		self.lon = msg.longitude

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
	
	def loop(self):
		rate = rospy.Rate(100) # 10hz

		while not rospy.is_shutdown():
			startTime = time.time()

			raw = self.bus.read_i2c_block_data(self.IMU_ADDR, self.EUL_X_LSB, 6)
			hdg,roll,pitch = struct.unpack('<hhh', bytearray(raw))

			hdg = hdg/16.0
			roll = roll/16.0
			pitch = pitch/16.0

			hdg = self.ConvertTo360Range(hdg-self.hdg_offset)

			#######################################
			### Live heading offset calibration ###
			#######################################
			if self.calib_flag:
				if self.get_latlon_once:
					lat_start = self.lat
					lon_start = self.lon
					self.get_latlon_once = False
					self.hdg_offset = 0.0
					hdg = self.ConvertTo360Range(hdg-self.hdg_offset)
					self.start_hdg = hdg
					rospy.loginfo("Start heading offset calibrating")

				lat_last = self.lat
				lon_last = self.lon

				self.ch7_from_high = True

			elif (self.calib_flag == False) and self.ch7_from_high:
				self.ch7_from_high = False
				line_hdg = self.get_bearing(lat_start, lon_start, lat_last, lon_last)
				self.hdg_offset = self.start_hdg - self.ConvertTo360Range(line_hdg)
				rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))

			else:
				self.get_latlon_once = True
				self.ch7_from_high = False


			self.compass_msg.data = [roll, pitch, hdg]


			self.compass_pub.publish(self.compass_msg)

			period = time.time() - startTime 
			# print("period", period)
			if period < 0.007:
				sleepMore = 0.007 - period
				time.sleep(sleepMore)

			rate.sleep()

if __name__ == "__main__":

	jmoab_compass = JMOAB_REF_COMPASS()
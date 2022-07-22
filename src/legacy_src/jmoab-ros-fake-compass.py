#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from sensor_msgs.msg import Imu, NavSatFix
import time
import struct
import numpy as np
import os
from std_msgs.msg import Float32MultiArray, Int32MultiArray

class JMOAB_FAKE_COMPASS:

	def __init__(self):

		rospy.init_node('jmoab_ros_fake_compass_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-FakeCompass node")
		rospy.loginfo("Publishing FakeCompass from IMU on /jmoab_compass, need to have heading_offset for correct true north")

		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/sbus_rc_ch", Int32MultiArray, self.sbus_callback)

		self.bus = SMBus(1)

		self.compass_pub = rospy.Publisher("/jmoab_compass", Float32MultiArray, queue_size=10)
		self.compass_msg = Float32MultiArray()


		## BNO055 address and registers
		self.IMU_ADDR = 0x28

		self.OPR_REG = 0x3d
		self.AXIS_MAP_CONFIG_REG = 0x41
		self.AXIS_MAP_SIGN_REG = 0x42

		self.QUA_w_LSB = 0x20
		self.QUA_w_MSB = 0x21
		self.QUA_x_LSB = 0x22
		self.QUA_x_MSB = 0x23
		self.QUA_y_LSB = 0x24
		self.QUA_y_MSB = 0x25
		self.QUA_z_LSB = 0x26
		self.QUA_z_MSB = 0x27

		# OPR_REG mode
		self.CONFIG_MODE = 0x00
		self.IMU_MODE = 0x08
		self.NDOF_MODE = 0x0C

		# AXIS REMAP
		self.REMAP_DEFAULT = 0x24

		# AXIS SIG
		self.SIGN_DEFAULT = 0x00	# if heatsink is down
		self.Z_REV = 0x01			# if heatsink is up
		self.YZ_REV = 0x03

		self.hdg_offset = 0.0

		# hdg_offset_file_name = "heading_offset.txt"
		# jmoab_calib_dir = "/home/nvidia/catkin_ws/src/jmoab-ros/example"
		# hdg_file_dir = os.path.join(jmoab_calib_dir, hdg_offset_file_name)
		# if os.path.exists(hdg_file_dir):
		# 	with open(hdg_file_dir, "r") as ff:

		# 		val = ff.read().strip()

		# 	if len(val) == 0:
		# 		self.hdg_offset = 0.0
		# 	else:
		# 		self.hdg_offset = float(val)

		rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))

		self.lat = 0.0
		self.lon = 0.0
		self.calib_flag = False
		self.get_latlon_once = True
		self.ch7_from_high = False
		
		self.imu_int()
		self.loop()

		rospy.spin()

	def gps_callback(self, msg):

		self.lat = msg.latitude
		self.lon = msg.longitude

	def sbus_callback(self, msg):
		if msg.data[6] > 1500:
			self.calib_flag = True
		else:
			self.calib_flag = False

	def imu_int(self):

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.CONFIG_MODE)
		time.sleep(0.019)	# 7ms for operating mode

		## comment this if no need config
		self.config_axis_remap()
		self.config_axis_sign()

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.IMU_MODE)
		time.sleep(0.019)	# 7ms for operating mode

	def config_axis_remap(self):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_CONFIG_REG, self.REMAP_DEFAULT)
		time.sleep(0.019)	# 19ms from any mode to config mode

	def config_axis_sign(self):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_SIGN_REG, self.SIGN_DEFAULT)
		time.sleep(0.019)	# 19ms from any mode to config mode

	def quaternion2rpy(self, qx, qy, qz, qw):
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

		y = np.arctan2(rot[1,0],rot[0,0])
		p = np.arcsin(-rot[2,0])
		r = np.arctan2(rot[2,1],rot[2,2])

		return r, p, y

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

			raw = self.bus.read_i2c_block_data(self.IMU_ADDR, self.QUA_w_LSB, 8)
			qw,qx,qy,qz = struct.unpack('<hhhh', bytearray(raw))

			qw = qw/16384.0
			qx = qx/16384.0
			qy = qy/16384.0
			qz = qz/16384.0

			roll, pitch, yaw = self.quaternion2rpy(qx, qy, qz, qw)

			roll = self.ConvertTo360Range(np.degrees(roll))
			pitch = self.ConvertTo360Range(np.degrees(pitch))

			yaw = self.ConvertTo360Range(-np.degrees(yaw) - self.hdg_offset)

			#######################################
			### Live heading offset calibration ###
			#######################################
			if self.calib_flag:
				if self.get_latlon_once:
					lat_start = self.lat
					lon_start = self.lon
					self.get_latlon_once = False
					self.hdg_offset = 0.0
					rospy.loginfo("Start heading offset calibrating")

				lat_last = self.lat
				lon_last = self.lon

				self.ch7_from_high = True

			elif (self.calib_flag == False) and self.ch7_from_high:
				self.ch7_from_high = False
				line_hdg = self.get_bearing(lat_start, lon_start, lat_last, lon_last)
				self.hdg_offset = yaw - self.ConvertTo360Range(line_hdg)
				rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))

			else:
				self.get_latlon_once = True
				self.ch7_from_high = False



			self.compass_msg.data = [roll, pitch, yaw]
			self.compass_pub.publish(self.compass_msg)

			period = time.time() - startTime 
			# print("period", period)
			if period < 0.007:
				sleepMore = 0.007 - period
				time.sleep(sleepMore)

if __name__ == "__main__":

	jmoab_fake_compass = JMOAB_FAKE_COMPASS()
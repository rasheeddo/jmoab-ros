#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from sensor_msgs.msg import Imu
import time
import struct
import numpy as np 
from std_msgs.msg import Float32MultiArray
import os
import argparse

class JMOAB_IMU(object):

	def __init__(self, addr, NS, frame_name):

		rospy.init_node('jmoab_ros_imu_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-IMU node")

		self.bus = SMBus(1)

		self.imu_pub = rospy.Publisher(self.namespace_attaching(NS, "/jmoab_imu_raw"), Imu, queue_size=10)
		self.imu_msg = Imu()

		self.ahrs_pub = rospy.Publisher(self.namespace_attaching(NS, "/jmoab_ahrs"), Float32MultiArray, queue_size=10)
		self.ahrs_msg = Float32MultiArray()
		rospy.loginfo("Publishing IMU Quaternion on /jmoab_imu_raw topic")

		## BNO055 address and registers
		if addr is None:
			self.IMU_ADDR = 0x28
		else:
			self.IMU_ADDR = int(addr, 0) # convert "0x29" a hex of string to integer

		self.frame_name = str(frame_name)

		self.OPR_REG = 0x3d
		self.AXIS_MAP_CONFIG_REG = 0x41
		self.AXIS_MAP_SIGN_REG = 0x42

		self.GYRO_x_LSB = 0x14

		self.EUL_Heading_LSB = 0x1a
		self.QUA_w_LSB = 0x20
		self.QUA_w_MSB = 0x21
		self.QUA_x_LSB = 0x22
		self.QUA_x_MSB = 0x23
		self.QUA_y_LSB = 0x24
		self.QUA_y_MSB = 0x25
		self.QUA_z_LSB = 0x26
		self.QUA_z_MSB = 0x27

		self.ACC_x_LSB = 0x28

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

			with_offset = True
		else:
			print("There is no calibration_offset.txt file!")
			print("Using default offset on the chip")
			with_offset = False

		
		self.imu_int(with_offset)
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


	def imu_int(self, write_offset):

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.CONFIG_MODE)
		time.sleep(0.019)	# 7ms for operating mode

		## comment this if no need config
		self.config_axis_remap()
		self.config_axis_sign()

		if write_offset:
			self.write_pre_calib()

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.IMU_MODE)
		time.sleep(0.019)	# 7ms for operating mode

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

	def config_axis_remap(self):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_CONFIG_REG, self.REMAP_DEFAULT)
		time.sleep(0.019)	# 19ms from any mode to config mode

	def config_axis_sign(self):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_SIGN_REG, self.SIGN_DEFAULT)
		time.sleep(0.019)	# 19ms from any mode to config mode

	def read_euler_angles(self):
		raw = self.bus.read_i2c_block_data(IMU_ADDR, EUL_Heading_LSB, 6)
		yaw,roll,pitch = struct.unpack('<hhh', bytearray(raw))
		yaw = yaw/16.0
		roll = roll/16.0
		pitch = pitch/16.0

		return [roll, pitch, yaw]

	def loop(self):
		rate = rospy.Rate(100) # 10hz

		while not rospy.is_shutdown():
			startTime = time.time()

			raw = self.bus.read_i2c_block_data(self.IMU_ADDR, self.EUL_Heading_LSB, 6)
			yaw,roll,pitch = struct.unpack('<hhh', bytearray(raw))
			yaw = yaw/16.0
			roll = roll/16.0
			pitch = pitch/16.0

			raw_quat = self.bus.read_i2c_block_data(self.IMU_ADDR, self.QUA_w_LSB, 8)
			qw,qx,qy,qz = struct.unpack('<hhhh', bytearray(raw_quat))

			qw = qw/16384.0
			qx = qx/16384.0
			qy = qy/16384.0
			qz = qz/16384.0

			raw_gyro = self.bus.read_i2c_block_data(self.IMU_ADDR, self.GYRO_x_LSB, 6)
			gx,gy,gz = struct.unpack('<hhh', bytearray(raw_gyro))

			gx = gx/16.0
			gy = gy/16.0
			gz = gz/16.0

			raw_acc = self.bus.read_i2c_block_data(self.IMU_ADDR, self.ACC_x_LSB, 6)
			ax,ay,az = struct.unpack('<hhh', bytearray(raw_acc))

			ax = ax/100.0
			ay = ay/100.0
			az = az/100.0

		

			# print("{:.3f}  {:.3f}  {:.3f}  {:.3f}".format(qw,qx,qy,qz))
			# print("gx: {:.3f}  gy: {:.3f}  gz: {:.3f}".format(gx,gy,gz))
			# print("ax: {:.3f}  ay: {:.3f}  az: {:.3f}".format(ax,ay,az))


			self.imu_msg.header.stamp = rospy.Time.now()
			self.imu_msg.header.frame_id = self.frame_name
			self.imu_msg.orientation.x = qx #roll
			self.imu_msg.orientation.y = qy #pitch
			self.imu_msg.orientation.z = qz #yaw
			self.imu_msg.orientation.w = qw 
			self.imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]

			self.imu_msg.angular_velocity.x = gx
			self.imu_msg.angular_velocity.y = gy
			self.imu_msg.angular_velocity.z = gz
			self.imu_msg.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]

			self.imu_msg.linear_acceleration.x = ax
			self.imu_msg.linear_acceleration.y = ay
			self.imu_msg.linear_acceleration.z = az
			self.imu_msg.linear_acceleration_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]

			self.ahrs_msg.data = [roll, pitch, yaw]

			self.imu_pub.publish(self.imu_msg)
			self.ahrs_pub.publish(self.ahrs_msg)

			period = time.time() - startTime 
			# print("period", period)
			if period < 0.007:
				sleepMore = 0.007 - period
				time.sleep(sleepMore)

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Compass node of jmoab-ros')
	parser.add_argument('--addr',
						help="i2c address of the imu, default is 0x28")
	parser.add_argument('--ns',
						help="a namespace in front of original topic")
	parser.add_argument('--frame_name',
						help="a frame name to use with TF")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	addr = args.addr
	ns = args.ns
	frame_name = args.frame_name

	if addr is None:
		print("Using default i2c address as 0x28")

	if frame_name is None:
		print("Using default frame name as 'base_link' ")
		frame_name = "base_link"


	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	jmoab_imu = JMOAB_IMU(addr, ns, frame_name)
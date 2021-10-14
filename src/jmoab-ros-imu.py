#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from sensor_msgs.msg import Imu
import time
import struct
import numpy as np
from sensor_msgs.msg import NavSatFix

class JMOAB_IMU:

	def __init__(self):

		rospy.init_node('jmoab_ros_imu_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-IMU node")

		self.bus = SMBus(1)

		self.imu_pub = rospy.Publisher("/jmoab_imu_raw", Imu, queue_size=10)
		self.imu_msg = Imu()
		rospy.loginfo("Publishing IMU Quaternion on /jmoab_imu_raw topic")

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

		
		self.imu_int()
		self.loop()

		rospy.spin()

	def imu_int(self):

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.CONFIG_MODE)
		time.sleep(0.019)	# 7ms for operating mode

		## comment this if no need config
		self.config_axis_remap()
		self.config_axis_sign()

		## change to operating mode
		self.bus.write_byte_data(self.IMU_ADDR, self.OPR_REG, self.NDOF_MODE)
		time.sleep(0.019)	# 7ms for operating mode

	def config_axis_remap(self):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_CONFIG_REG, self.REMAP_DEFAULT)
		time.sleep(0.019)	# 19ms from any mode to config mode

	def config_axis_sign(self):
		self.bus.write_byte_data(self.IMU_ADDR, self.AXIS_MAP_SIGN_REG, self.SIGN_DEFAULT)
		time.sleep(0.019)	# 19ms from any mode to config mode

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

			# print("{:.3f}  {:.3f}  {:.3f}  {:.3f}".format(qw,qx,qy,qz))

			self.imu_msg.header.stamp = rospy.Time.now()
			self.imu_msg.header.frame_id = 'base_link'
			self.imu_msg.orientation.x = qx #roll
			self.imu_msg.orientation.y = qy #pitch
			self.imu_msg.orientation.z = qz #yaw
			self.imu_msg.orientation.w = qw 

			self.imu_pub.publish(self.imu_msg)

			period = time.time() - startTime 
			# print("period", period)
			if period < 0.007:
				sleepMore = 0.007 - period
				time.sleep(sleepMore)

if __name__ == "__main__":

	jmoab_imu = JMOAB_IMU()
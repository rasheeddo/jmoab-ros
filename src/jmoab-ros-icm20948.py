#! /usr/bin/env python
import rospy
import rospkg
from icm20948 import ICM20948
from sensor_msgs.msg import Imu, MagneticField
import time
import struct
import numpy as np
from sensor_msgs.msg import NavSatFix
import os
import argparse

class JMOAB_ICM20948(object):

	def __init__(self, addr, NS):

		rospy.init_node('jmoab_icm20948_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ICM20948 node")

		self.imu_pub = rospy.Publisher(self.namespace_attaching(NS, "/imu/data_raw"), Imu, queue_size=10)
		self.imu_msg = Imu()

		self.mag_pub = rospy.Publisher(self.namespace_attaching(NS, "/imu/mag"), MagneticField, queue_size=10)
		self.mag_msg = MagneticField()

		rospy.loginfo("Publishing IMU Quaternion on /imu/data_raw, /imu/mag topic")

		## BNO055 address and registers
		if addr is None:
			self.IMU_ADDR = 0x69
		else:
			self.IMU_ADDR = int(addr, 0) # convert "0x29" a hex of string to integer

		self.imu = ICM20948(i2c_addr=self.IMU_ADDR)
		self.imu.bank(2)
		self.imu.set_gyro_sample_rate(100)
		self.imu.set_gyro_low_pass(enabled=True, mode=5)
		self.imu.set_gyro_full_scale(250)
		self.imu.set_accelerometer_sample_rate(125)
		self.imu.set_accelerometer_low_pass(enabled=True, mode=5)
		self.imu.set_accelerometer_full_scale(16)

		self.pre_calib = []

		rospack = rospkg.RosPack()
		jmoab_ros_path = rospack.get_path("jmoab-ros")

		rospy.loginfo("Read icm20948_offset.txt file")
		calib_file_name = "icm20948_offset.txt"
		calib_file_dir = os.path.join(jmoab_ros_path, "example", calib_file_name)
		if os.path.exists(calib_file_dir):
			with open(calib_file_dir, "r") as f:
				for line in f:
					self.pre_calib.append(float(line.strip()))

			with_offset = True
			self.ax_scale = self.pre_calib[0]
			self.ax_bias = self.pre_calib[1]
			self.ay_scale = self.pre_calib[2]
			self.ay_bias = self.pre_calib[3]
			self.az_scale = self.pre_calib[4]
			self.az_bias = self.pre_calib[5]
			self.gx_bias = self.pre_calib[6]
			self.gy_bias = self.pre_calib[7]
			self.gz_bias = self.pre_calib[8]
			self.mx_scale = self.pre_calib[9]
			self.mx_bias = self.pre_calib[10]
			self.my_scale = self.pre_calib[11]
			self.my_bias = self.pre_calib[12]
			self.mz_scale = self.pre_calib[13]
			self.mz_bias = self.pre_calib[14]

		else:
			print("There is no icm20948_offset.txt file!")
			with_offset = False
			self.ax_scale = 1.0
			self.ax_bias = 0.0
			self.ay_scale = 1.0
			self.ay_bias = 0.0
			self.az_scale = 1.0
			self.az_bias = 0.0
			self.gx_bias = 0.0
			self.gy_bias = 0.0
			self.gz_bias = 0.0
			self.mx_scale = 1.0
			self.mx_bias = 0.0
			self.my_scale = 1.0
			self.my_bias = 0.0
			self.mz_scale = 1.0
			self.mz_bias = 0.0

		rospy.loginfo("Using sensor scales/biases as following")
		rospy.loginfo("ax_scale: {:.4f} | ax_bias: {:.4f}".format(self.ax_scale, self.ax_bias))
		rospy.loginfo("ay_scale: {:.4f} | ay_bias: {:.4f}".format(self.ay_scale, self.ay_bias))
		rospy.loginfo("az_scale: {:.4f} | az_bias: {:.4f}".format(self.az_scale, self.az_bias))
		rospy.loginfo("gx_bias: {:.4f}".format(self.gx_bias))
		rospy.loginfo("gy_bias: {:.4f}".format(self.gy_bias))
		rospy.loginfo("gz_bias: {:.4f}".format(self.gz_bias))
		rospy.loginfo("mx_scale: {:.4f} | mx_bias: {:.4f}".format(self.mx_scale, self.mx_bias))
		rospy.loginfo("my_scale: {:.4f} | my_bias: {:.4f}".format(self.my_scale, self.my_bias))
		rospy.loginfo("mz_scale: {:.4f} | mz_bias: {:.4f}".format(self.mz_scale, self.mz_bias))
		
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

	def loop(self):
		rate = rospy.Rate(100) # 10hz

		while not rospy.is_shutdown():
			startTime = time.time()

			mx, my, mz = self.imu.read_magnetometer_data()
			ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()

			ax = (ax*self.ax_scale) + self.ax_bias
			ay = (ay*self.ay_scale) + self.ay_bias
			az = (az*self.az_scale) + self.az_bias

			gx = gx - self.gx_bias
			gy = gy - self.gy_bias
			gz = gz - self.gz_bias

			mx = self.mx_scale*(mx-self.mx_bias)
			my = self.my_scale*(my-self.my_bias)
			mz = self.mz_scale*(mz-self.mz_bias)


			ax = ax*9.81
			ay = ay*9.81
			az = az*9.81


			# print("{:.3f}  {:.3f}  {:.3f}  {:.3f}".format(qw,qx,qy,qz))
			# print("gx: {:.3f}  gy: {:.3f}  gz: {:.3f}".format(gx,gy,gz))
			# print("ax: {:.3f}  ay: {:.3f}  az: {:.3f}".format(ax,ay,az))


			self.imu_msg.header.stamp = rospy.Time.now()
			self.imu_msg.header.frame_id = "imu_link"
			self.imu_msg.orientation.x = 0.0
			self.imu_msg.orientation.y = 0.0
			self.imu_msg.orientation.z = 0.0
			self.imu_msg.orientation.w = 0.0
			self.imu_msg.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

			self.imu_msg.angular_velocity.x = np.radians(gx)
			self.imu_msg.angular_velocity.y = np.radians(gy)
			self.imu_msg.angular_velocity.z = np.radians(gz)
			self.imu_msg.angular_velocity_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

			self.imu_msg.linear_acceleration.x = ax
			self.imu_msg.linear_acceleration.y = ay
			self.imu_msg.linear_acceleration.z = az
			self.imu_msg.linear_acceleration_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

			self.mag_msg.header.stamp = rospy.Time.now()
			self.mag_msg.header.frame_id = "compass_link"
			self.mag_msg.magnetic_field.x = mx*1e-6
			self.mag_msg.magnetic_field.y = my*1e-6
			self.mag_msg.magnetic_field.z = mz*1e-6


			self.imu_pub.publish(self.imu_msg)
			self.mag_pub.publish(self.mag_msg)

			rate.sleep()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Compass node of jmoab-ros')
	parser.add_argument('--addr',
						help="i2c address of the imu, default is 0x28")
	parser.add_argument('--ns',
						help="a namespace in front of original topic")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	addr = args.addr
	ns = args.ns

	if addr is None:
		print("Using default i2c address as 0x69")

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	jmoab_imu = JMOAB_ICM20948(addr, ns)
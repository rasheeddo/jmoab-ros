#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Int32MultiArray, Float32MultiArray
import time
import struct
import numpy as np
import sys
import serial
import json

class JMOAB_HALL_WHEELS(object):

	def __init__(self, ser_port):

		rospy.init_node('jmoab_ros_hall_wheels_odom', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-HALL-WHEELS-ODOM node")

		self.wheels_rpm_pub = rospy.Publisher("/wheels_rpm", Float32MultiArray, queue_size=10)
		self.wheels_rpm_msg = Float32MultiArray()
		rospy.loginfo("Publishing hall wheels odometry on /wheels_rpm topic")
		rospy.loginfo("Slowest RPM you can get is around 6 RPM")

		self.ser = serial.Serial(ser_port, 115200)

		self.rpm_L = 0.0
		self.rpm_R = 0.0
		self.array_len = 20
		self.last_count_L = 0
		self.last_count_R = 0
		self.count_change_timeout = 8.0
		
		self.loop()

		rospy.spin()


	def MovingAverage(self, data_raw, data_array, array_size):

		if len(data_array) < array_size:
			data_array = np.append(data_array, data_raw)
		else:
			data_array = np.delete(data_array, 0)
			data_array = np.append(data_array, data_raw)
		data_ave = np.average(data_array)
		return data_ave, data_array


	def loop(self):
		rate = rospy.Rate(100) # 10hz

		rpm_R_array = np.array([])
		rpm_L_array = np.array([])
		prev_rpm_L = 0.0
		prev_rpm_R = 0.0
		count_last_change_L = time.time()
		count_last_change_R = time.time()

		while not rospy.is_shutdown():
			try:
				line = self.ser.readline()
				string_data = line.decode().strip()
				load_data = json.loads(string_data)
				rpm_L = load_data["rpm_L"]
				rpm_R = load_data["rpm_R"]


				print("rpm_L: {:.3f} | rpm_R: {:.3f}".format(load_data["rpm_L"], load_data["rpm_R"]))


				# rpm_L_ave, rpm_L_array = self.MovingAverage(rpm_L, rpm_L_array, self.array_len)
				# rpm_R_ave, rpm_R_array = self.MovingAverage(rpm_R, rpm_R_array, self.array_len)

				# self.wheel_odom_msg.data = [rpm_L_ave, rpm_R_ave]
				self.wheels_rpm_msg.data = [rpm_L, rpm_R]  
				self.wheels_rpm_pub.publish(self.wheels_rpm_msg)

				prev_rpm_L = rpm_L
				prev_rpm_R = rpm_R

			except Exception as e:
				print(e)
				print("Failed to parse")
				pass

			rate.sleep()

if __name__ == "__main__":

	if len(sys.argv) < 2:
		print("Please specify Arduino serial device port")
		print("usage: rosrun jmoab-ros jmoab-ros-wheels-rpm.py /dev/ttyUSBx")
	else:
		ser_port = str(sys.argv[1])
		jhw = JMOAB_HALL_WHEELS(ser_port)

	
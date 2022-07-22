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

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.sbus_cmd_callback)
		rospy.Subscriber("/sbus_rc_ch", Int32MultiArray, self.sbus_rc_callback)

		self.wheel_odom_pub = rospy.Publisher("/wheel_odom", Float32MultiArray, queue_size=10)
		self.wheel_odom_msg = Float32MultiArray()
		rospy.loginfo("Publishing hall wheels odometry on /wheel_odom topic")

		self.robot_mode = "MANUAL"
		self.sbus_rc_throttle = 1024
		self.sbus_rc_steering = 1024
		self.sbus_auto_throttle = 1024
		self.sbus_auto_steering = 1024
		self.sbus_mid = 1024

		self.sbus_thresh = 76
		self.deadband_range = range((self.sbus_mid-76), (self.sbus_mid+76))

		self.sbus_cmd_timestamp = time.time()

		self.ser = serial.Serial(ser_port, 115200)

		self.rpm_L = 0.0
		self.rpm_R = 0.0
		self.array_len = 20
		self.last_count_L = 0
		self.last_count_R = 0
		self.count_change_timeout = 8.0
		
		self.loop()

		rospy.spin()


	def sbus_rc_callback(self, msg):

		# T3PV, chnage mode is ch3 (msg.data[2])
		# T10J, change mode is ch5 (msg.data[4])

		if msg.data[4] < 1500:
			self.robot_mode = "MANUAL"
			self.sbus_rc_steering = msg.data[0]
			self.sbus_rc_throttle = msg.data[1]
		else:
			self.robot_mode = "AUTO"
			self.sbus_rc_steering = 1024
			self.sbus_rc_throttle = 1024

	def sbus_cmd_callback(self, msg):

		self.sbus_auto_steering = msg.data[0]
		self.sbus_auto_throttle = msg.data[1]
		self.sbus_cmd_timestamp = time.time()


	def MovingAverage(self, data_raw, data_array, array_size):

		if len(data_array) < array_size:
			data_array = np.append(data_array, data_raw)
		else:
			data_array = np.delete(data_array, 0)
			data_array = np.append(data_array, data_raw)
		data_ave = np.average(data_array)
		return data_ave, data_array


	def loop(self):
		rate = rospy.Rate(20) # 10hz

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
				if self.robot_mode == "MANUAL":
					if (self.sbus_rc_steering in self.deadband_range) and (self.sbus_rc_throttle in self.deadband_range):

						rpm_L = 0.0
						rpm_R = 0.0
						rpm_R_array = np.array([])
						rpm_L_array = np.array([])


					else:
						if load_data["count_L"] != self.last_count_L:
							rpm_L = load_data["rpm_L"]
							self.last_count_L = load_data["count_L"]
							count_last_change_L = time.time()
						else:
							if (time.time() - count_last_change_L) > self.count_change_timeout:
								## in case of very very slow rpm, even pushing throttle & steering
								rpm_L = 0.0
								rpm_L_array = np.array([])
							else:
								rpm_L = prev_rpm_L



						if load_data["count_R"] != self.last_count_R:
							rpm_R = load_data["rpm_R"]
							self.last_count_R = load_data["count_R"]
							count_last_change_R = time.time()
						else:
							if (time.time() - count_last_change_R) > self.count_change_timeout:
								## in case of very very slow rpm, even pushing throttle & steering
								rpm_R = 0.0
								rpm_R_array = np.array([])
							else:
								rpm_R = prev_rpm_R



				elif self.robot_mode == "AUTO":
					sbus_cmd_timeout = (time.time() - self.sbus_cmd_timestamp)
					# print("sbus_cmd_timeout", sbus_cmd_timeout)
					if (self.sbus_auto_steering in self.deadband_range) and (self.sbus_auto_throttle in self.deadband_range) or (sbus_cmd_timeout > 2.0):

						rpm_L = 0.0
						rpm_R = 0.0
						rpm_R_array = np.array([])
						rpm_L_array = np.array([])

					else:
						if load_data["count_L"] != self.last_count_L:
							rpm_L = load_data["rpm_L"]
							self.last_count_L = load_data["count_L"]
							count_last_change_L = time.time()
						else:
							if (time.time() - count_last_change_L) > self.count_change_timeout:
								## in case of very very slow rpm, even pushing throttle & steering
								rpm_L = 0.0
								rpm_L_array = np.array([])
							else:
								rpm_L = load_data["rpm_L"]

						if load_data["count_R"] != self.last_count_R:
							rpm_R = load_data["rpm_R"]
							self.last_count_R = load_data["count_R"]
							count_last_change_R = time.time()
						else:
							if (time.time() - count_last_change_R) > self.count_change_timeout:
								## in case of very very slow rpm, even pushing throttle & steering
								rpm_R = 0.0
								rpm_R_array = np.array([])
							else:		
								rpm_R = load_data["rpm_R"]

				else:
					rpm_L = 0.0
					rpm_R = 0.0
					rpm_R_array = np.array([])
					rpm_L_array = np.array([])


				print("rpm_R: {:.3f} | count_R: {:d} | rpm_L: {:.3f} | count_L: {:d}".format(\
				load_data["rpm_R"], load_data["count_R"],\
				 load_data["rpm_L"], load_data["count_L"]))


				rpm_L_ave, rpm_L_array = self.MovingAverage(rpm_L, rpm_L_array, self.array_len)
				rpm_R_ave, rpm_R_array = self.MovingAverage(rpm_R, rpm_R_array, self.array_len)

				self.wheel_odom_msg.data = [rpm_L_ave, rpm_R_ave] 
				self.wheel_odom_pub.publish(self.wheel_odom_msg)

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
		print("usage: rosrun jmoab-ros jmoab-ros-hallWheelsOdom.py /dev/ttyUSBx")
	else:
		ser_port = str(sys.argv[1])
		jhw = JMOAB_HALL_WHEELS(ser_port)

	
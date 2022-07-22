#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Int32MultiArray, Float32MultiArray, Int8
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import struct
import numpy as np
import sys
import serial
import json

class JMOAB_HALL_WHEELS(object):

	def __init__(self, ser_port):

		rospy.init_node('jmoab_ros_wheels_rpm', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-WHEELS-RPM node")

		self.wheels_rpm_pub = rospy.Publisher("/wheels_rpm", Float32MultiArray, queue_size=10)
		self.wheels_rpm_msg = Float32MultiArray()
		rospy.loginfo("Publishing hall wheels odometry on /wheels_rpm topic")
		rospy.loginfo("Slowest RPM you can get is around 6 RPM")

		self.wheels_speed_pub = rospy.Publisher("/wheels_speed", Float32MultiArray, queue_size=10)
		self.wheels_speed_msg = Float32MultiArray()

		# self.wheels_speed_est_pub = rospy.Publisher("/wheels_speed_est", Float32MultiArray, queue_size=10)
		# self.wheels_speed_est_msg = Float32MultiArray()

		self.ser = serial.Serial(ser_port, 115200)

		self.rpm_L = 0.0
		self.rpm_R = 0.0
		self.array_len = 20
		self.last_count_L = 0
		self.last_count_R = 0
		self.count_change_timeout = 8.0

		## Kalman filter of speed estimate
		self.VL_est = 0.0 # first guess
		self.VL_predict = self.VL_est
		self.VL_error_est = 0.1 
		self.VL_error_mea = 0.1 # a variance of

		self.VR_est = 0.0 # first guess
		self.VR_predict = self.VR_est
		self.VR_error_est = 0.1 
		self.VR_error_mea = 0.1 # a variance of

		self.sbus_steering_stick = 1024
		self.sbus_throttle_stick = 1024
		self.sbus_cmd_left = 1024
		self.sbus_cmd_right = 1024
		self.cart_mode = 0

		self.L = 0.52	# wheel's base width
		self.dt = 0.01

		self.br = tf2_ros.TransformBroadcaster()
		self.t = TransformStamped()

		self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
		self.odom_msg = Odometry()

		rospy.Subscriber("/sbus_rc_ch", Int32MultiArray, self.sbus_rc_callback)
		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.sbus_cmd_callback)
		rospy.Subscriber("/atcart_mode", Int8, self.atcart_mode_callback)
		
		self.loop()

		rospy.spin()

	def sbus_rc_callback(self, msg):

		self.sbus_steering_stick = msg.data[0]
		self.sbus_throttle_stick = msg.data[1]

	def sbus_cmd_callback(self, msg):

		self.sbus_cmd_left = msg.data[0]
		self.sbus_cmd_right = msg.data[1]

	def atcart_mode_callback(self, msg):
		self.cart_mode = msg.data

	def MovingAverage(self, data_raw, data_array, array_size):

		if len(data_array) < array_size:
			data_array = np.append(data_array, data_raw)
		else:
			data_array = np.delete(data_array, 0)
			data_array = np.append(data_array, data_raw)
		data_ave = np.average(data_array)
		return data_ave, data_array

	def kalman_filter(self, L_mea, R_mea, prev_L_est, prev_R_est, prev_L_error_est, prev_R_error_est):

		## reference
		## https://www.kalmanfilter.net/kalman1d.html

		######################
		## state estimation ##
		######################
		KG_L = prev_L_error_est/(prev_L_error_est + self.VL_error_mea)
		cur_L_est = prev_L_est + KG_L*(L_mea - prev_L_est)
		cur_L_error_est = (1.0 - KG_L)*(prev_L_error_est)

		KG_R = prev_R_error_est/(prev_R_error_est + self.VR_error_mea)
		cur_R_est = prev_R_est + KG_R*(R_mea - prev_R_est)
		cur_R_error_est = (1.0 - KG_R)*(prev_R_error_est)

		################
		## prediction ##
		################
		self.VL_error_est = cur_L_error_est + 0.01 
		self.VR_error_est = cur_R_error_est + 0.01 
		
		# hdg_offset is not dynamic behaviour, so predicted state is constant as estimated state
		self.VL_predict = cur_L_est
		self.VR_predict = cur_R_est    

		return cur_L_est, cur_R_est
	

	def loop(self):
		rate = rospy.Rate(100) # 10hz

		rpm_R_array = np.array([])
		rpm_L_array = np.array([])
		prev_rpm_L = 0.0
		prev_rpm_R = 0.0
		prev_count_L = 0
		prev_count_R = 0
		count_last_change_L = time.time()
		count_last_change_R = time.time()
		counter_L = 0
		counter_R = 0
		theta = 0.0
		x = 0.0
		y = 0.0
		period = 0.01
		dt = 0.017

		while not rospy.is_shutdown():
			startTime = time.time()

			# if period > self.dt:
			# 	dt = 0.01 + (period - self.dt)
			# else:
			# 	dt = self.dt

			try:
				line = self.ser.readline()
				string_data = line.decode().strip()
				load_data = json.loads(string_data)
				rpm_L = load_data["rpm_L"]
				rpm_R = load_data["rpm_R"]
				counter_L = load_data["counter_L"]
				counter_R = load_data["counter_R"]
				VL = load_data["VL"]
				VR = load_data["VR"]

				if self.cart_mode == 1:
					if (994 < self.sbus_throttle_stick < 1054) and (994 < self.sbus_steering_stick < 1054):
						VL = 0.0
						VR = 0.0
				elif self.cart_mode == 2:
					if (968 < self.sbus_cmd_left< 1093):
						VL = 0.0

					if (968 < self.sbus_cmd_right < 1093):
						VR = 0.0



				self.VL_est , self.VR_est = self.kalman_filter(\
					VL, VR, self.VL_predict, self.VR_predict, self.VL_error_est, self.VR_error_est)



				# vl = round(self.VL_est, 3)
				# vr = round(self.VR_est, 3)
				vl = round(VL, 3)
				vr = round(VR, 3)

				if (vl > 0.0) and (vr < 0.0) and (abs(vl) == abs(vr)):
					## rotatiing CW
					V = -vl
					Wz = 2.0*V/self.L
					theta = theta + Wz*dt

					path = "skid_right"

				elif (vr > 0.0) and (vl < 0.0) and (abs(vl) == abs(vr)):
					## rotatiing CCW
					V = vr
					Wz = 2.0*V/self.L
					theta = theta + Wz*dt

					path = "skid_left"

				elif abs(vl) > abs(vr):
					## curving CW
					V = (vl + vr)/2.0
					Wz = (vl-vr)/self.L
					R_ICC = (self.L/2.0)*((vl+vr)/(vl-vr))

					x = x - R_ICC*np.sin(theta) + R_ICC*np.sin(theta + Wz*dt)
					y = y + R_ICC*np.cos(theta) - R_ICC*np.cos(theta + Wz*dt)
					theta = theta - Wz*dt

					path = "curve_right"

				elif abs(vl) < abs(vr):
					## curving CCW
					V = (vl + vr)/2.0
					Wz = (vr-vl)/self.L
					R_ICC = (self.L/2.0)*((vr+vl)/(vr-vl))

					x = x - R_ICC*np.sin(theta) + R_ICC*np.sin(theta + Wz*dt)
					y = y + R_ICC*np.cos(theta) - R_ICC*np.cos(theta + Wz*dt)
					theta = theta + Wz*dt

					path = "curve_left"

				elif vl == vr:
					V = (vl + vr)/2.0
					Wz = 0.0
					x = x + V*np.cos(theta)*dt
					y = y + V*np.sin(theta)*dt
					theta = theta
					path = "straight"

				else:
					V = 0.0
					Wz = 0.0
					path = "unknown1"


				# vx = (self.VL_est + self.VR_est)/2.0
				# vx = (VL + VR)/2.0
				# vy = 0.0
				# vth = (VR - VL)/self.cart_width 

				# dx = vx * np.cos(theta) * period	#self.dt
				# dy = vx * np.sin(theta) * period	#self.dt
				# dtheta = vth*period	#self.dt

				# x += dx
				# y += dy
				# theta += dtheta

				q = quaternion_from_euler(0,0, theta)
				# construct tf
				self.t.header.frame_id = "odom" 
				self.t.header.stamp = rospy.Time.now()
				self.t.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
				self.t.transform.translation.x = x
				self.t.transform.translation.y = y
				self.t.transform.translation.z = 0.0

				self.t.transform.rotation.x = q[0]
				self.t.transform.rotation.y = q[1]
				self.t.transform.rotation.z = q[2]
				self.t.transform.rotation.w = q[3]
				self.br.sendTransform(self.t)

				self.odom_msg.header.stamp = rospy.Time.now()
				self.odom_msg.header.frame_id = "odom"
				self.odom_msg.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
				self.odom_msg.pose.pose.position.x = x
				self.odom_msg.pose.pose.position.y = y
				self.odom_msg.pose.pose.position.z = 0.0
				self.odom_msg.pose.pose.orientation.x = q[0]
				self.odom_msg.pose.pose.orientation.y = q[1]
				self.odom_msg.pose.pose.orientation.z = q[2]
				self.odom_msg.pose.pose.orientation.w = q[3]
				self.odom_msg.pose.covariance[0] = 0.0001
				self.odom_msg.pose.covariance[7] = 0.0001
				self.odom_msg.pose.covariance[14] = 0.000001	#1e12
				self.odom_msg.pose.covariance[21] = 0.000001	#1e12
				self.odom_msg.pose.covariance[28] = 0.000001	#1e12
				self.odom_msg.pose.covariance[35] = 0.0001
				self.odom_msg.twist.twist.linear.x = V
				self.odom_msg.twist.twist.linear.y = 0.0
				# self.odom_msg.twist.covariance[0] = 0.001
				# self.odom_msg.twist.covariance[7] = 0.001
				# self.odom_msg.twist.covariance[14] = 1e9
				# self.odom_msg.twist.covariance[21] = 1e9
				# self.odom_msg.twist.covariance[28] = 1e9
				# self.odom_msg.twist.covariance[35] = 0.001
				self.odom_msg.twist.twist.angular.z = Wz

				self.odom_pub.publish(self.odom_msg)

				# self.wheel_odom_msg.data = [rpm_L_ave, rpm_R_ave]
				self.wheels_rpm_msg.data = [rpm_L, rpm_R]  
				self.wheels_rpm_pub.publish(self.wheels_rpm_msg)

				self.wheels_speed_msg.data = [VL, VR]  
				self.wheels_speed_pub.publish(self.wheels_speed_msg)

				# self.wheels_speed_est_msg.data = [self.VL_est, self.VR_est]  
				# self.wheels_speed_est_pub.publish(self.wheels_speed_est_msg)

				# print("rpm_L: {:.3f} | rpm_R: {:.3f} | counter_L: {:d} | counter_R: {:d}".format(\
				# 	load_data["rpm_L"], load_data["rpm_R"], counter_L, counter_R))

				# print("rpm_L: {:.3f} | rpm_R: {:.3f} | VL: {:.3f} | VR: {:.3f} | VL_est: {:.3f} | VR_est: {:.3f}".format(\
				# 	rpm_L, rpm_R, VL, VR, self.VL_est, self.VR_est))

				#print("{:} | vl: {:.3f} | vr: {:.3f} | V: {:.3f} | Wz: {:.3f} | x: {:.3f} | y: {:.3f} | th: {:.2f} | dt: {:.4f}".format(\
				#	path, vl, vr, V, Wz, x, y, np.degrees(theta), dt))

				prev_rpm_L = rpm_L
				prev_rpm_R = rpm_R

				period = time.time() - startTime


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

	
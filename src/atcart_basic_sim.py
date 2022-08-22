#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, UInt8, Int16MultiArray
from sensor_msgs.msg import Joy
import time
import argparse

class ATCartSim(object):

	def __init__(self, NS):

		rospy.init_node('jmoab_ros_vel_2_sbus', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS ATCart Simulation node")
		rospy.loginfo("Please run 'rosrun joy joy_node' as a transmitter device")

		self.cmd_steering = 1024
		self.cmd_throttle = 1024

		self.callback_timeout = 0.5 # second
		self.cmd_vel_callback_timestamp = time.time()
		self.atcart_wheels_cmd_callback_timestamp = time.time()

		self.analog_steering = 1024
		self.analog_throttle = 1024

		self.sbus_mid = 1024
		self.sbus_min = 368
		self.sbus_max = 1680

		self.str_percent = 0.0
		self.thr_percent = 0.0

		self.vx_max = 2.0
		self.wz_max = 2.0

		self.cart_mode = 0
		self.mode_name = "HOLD"

		## -143	 RPM -->  368
		## 4.5   RPM -->  1100
		## 143   RPM -->  1680

		self.prev_y = 0.0

		self.vx = 0.0
		self.wz = 0.0

		self.left_percent_cmd = 0.0
		self.right_percent_cmd = 0.0

		self.ch1_sim = 1024
		self.ch2_sim = 1024
		self.ch5_sim = 1024
		self.ch7_sim = 1024
		self.ch8_sim = 1024
		self.ch9_sim = 1024

		## Pub/Sub ##
		cmd_vel_topic = self.namespace_attaching(NS, "/cmd_vel")
		atcart_mode_topic = self.namespace_attaching(NS, "/jmoab/cart_mode")
		atcart_mode_cmd_topic = self.namespace_attaching(NS, "/jmoab/cart_mode_cmd")
		atcart_sim_wheels_cmd_topic = self.namespace_attaching(NS, "/atcart_sim/wheels_cmd")
		atcart_wheels_cmd_topic = self.namespace_attaching(NS, "/jmoab/wheels_cmd")
		sbus_rc_ch_sim_topic = self.namespace_attaching(NS, "/jmoab/sbus_rc_ch")

		self.atcart_wheels_cmd_pub = rospy.Publisher(atcart_sim_wheels_cmd_topic, Float32MultiArray, queue_size=10)
		self.atcart_wheels_cmd_msg = Float32MultiArray()
		
		self.atcart_mode_pub = rospy.Publisher(atcart_mode_topic, UInt8, queue_size=10)
		self.atcart_mode = UInt8()

		self.sbus_rc_ch_sim_pub = rospy.Publisher(sbus_rc_ch_sim_topic, Int16MultiArray, queue_size=10)
		self.sbus_rc_ch_sim_msg = Int16MultiArray()

		rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)
		rospy.Subscriber(atcart_wheels_cmd_topic, Float32MultiArray, self.atcart_wheels_cmd_callback)
		rospy.Subscriber(atcart_mode_cmd_topic, UInt8, self.cart_mode_callack)
		rospy.Subscriber("joy", Joy, self.joy_callback)
		
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

	def cmd_vel_callback(self, msg):

		if msg.linear.x > self.vx_max:
			vx = self.vx_max
		elif msg.linear.x < -self.vx_max:
			vx = -self.vx_max
		else:
			vx = msg.linear.x

		if msg.angular.z > self.wz_max:
			wz = self.wz_max
		elif msg.angular.z < -self.vx_max:
			wz = -self.wz_max
		else:
			wz = msg.angular.z

		self.vx = vx
		self.wz = wz

		self.cmd_vel_callback_timestamp = time.time()

	def atcart_wheels_cmd_callback(self, msg):

		if msg.data[0] > 100.0:
			left = 100.0
		elif msg.data[0] < -100.0:
			left = -100.0
		else:
			left = msg.data[0]

		if msg.data[1] > 100.0:
			right = 100.0
		elif msg.data[1] < -100.0:
			right = -100.0
		else:
			right = msg.data[1]

		self.left_percent_cmd = left
		self.right_percent_cmd = right

		self.atcart_wheels_cmd_callback_timestamp = time.time()

	def cart_mode_callack(self, msg):
		if msg.data == 0:
			self.cart_mode = 0
			self.mode_name = "HOLD"
		elif msg.data == 1:
			self.cart_mode = 1
			self.mode_name = "MANUAL"
		elif msg.data == 2:
			self.cart_mode = 2
			self.mode_name = "AUTO"


	def joy_callback(self, msg):

		if msg.buttons[0] == 1:
			self.cart_mode = 2
			self.mode_name = "AUTO"
			self.ch5_sim = 1904
		elif msg.buttons[1] == 1:
			self.cart_mode = 0
			self.mode_name = "HOLD"
			self.ch5_sim = 144
		elif msg.buttons[2] == 1:
			self.cart_mode = 1
			self.mode_name = "MANUAL"
			self.ch5_sim = 1024

		## back
		if msg.buttons[7] == 1:
			self.ch7_sim = 144
		## Logicool
		elif msg.buttons[8] == 1:
			self.ch7_sim = 1024
		## start
		elif msg.buttons[6] == 1:
			self.ch7_sim = 1904

		## Y
		if msg.buttons[3] == 1:
			self.ch8_sim = 1904
		else:
			self.ch8_sim = 144

		## RB
		if msg.buttons[5] == 1:
			self.ch9_sim = 1904
		## LB
		elif msg.buttons[4] == 1:
			self.ch9_sim = 144

		self.ch1_sim = int(self.map(msg.axes[3], -1.0, 1.0, 1680.0, 368.0))
		self.ch2_sim = int(self.map(msg.axes[1], -1.0, 1.0, 368.0, 1680.0))

		self.str_percent = self.map(msg.axes[3], -1.0, 1.0, 100.0, -100.0)
		self.thr_percent = self.map(msg.axes[1], -1.0, 1.0, -100.0, 100.0)


	def map(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter

		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		return out

	def xy_mixing(self, x, y):
		## x, y must be in the range of -100 to 100

		left = y+x
		right = y-x

		diff = abs(x) - abs(y)

		if (left < 0.0):
			left = left - abs(diff)
		else:
			left = left + abs(diff)

		if (right < 0.0):
			right = right - abs(diff)
		else:
			right = right + abs(diff)

		if (self.prev_y < 0.0):
			swap = left
			left = right
			right = swap
		
		self.prev_y = y

		## left and right are in -200 to 200 ranges

		return left, right


	def loop(self):

		rate = rospy.Rate(50)

		sbus_steering  = 1024
		sbus_throttle = 1024

		while not rospy.is_shutdown():

			self.sbus_rc_ch_sim_msg.data = [self.ch1_sim, self.ch2_sim, 1024, 1024, self.ch5_sim, 1024, self.ch7_sim, self.ch8_sim, self.ch9_sim]
			self.sbus_rc_ch_sim_pub.publish(self.sbus_rc_ch_sim_msg)

			self.atcart_mode = self.cart_mode
			self.atcart_mode_pub.publish(self.atcart_mode)



			if self.mode_name == "AUTO":
				cmd_vel_period = time.time() - self.cmd_vel_callback_timestamp
				wheels_cmd_period = time.time() - self.atcart_wheels_cmd_callback_timestamp

				## 1st priority is cmd_vel
				if cmd_vel_period < 0.5:
					x_percent = self.map(self.wz, -self.wz_max, self.wz_max, 100.0, -100.0)
					y_percent = self.map(self.vx, -self.vx_max, self.vx_max, -100.0, 100.0)

					left_200_per, right_200_per = self.xy_mixing(x_percent, y_percent)

					left_100_per = self.map(left_200_per, -200.0, 200.0, -100.0, 100.0)
					right_100_per = self.map(right_200_per, -200.0, 200.0, -100.0, 100.0)

				## 2nd priority is /jmoab/wheels_cmd
				elif wheels_cmd_period < 0.5:
					left_100_per = self.left_percent_cmd
					right_100_per = self.right_percent_cmd
				else:
					left_100_per = 0.0
					right_100_per = 0.0

			elif self.mode_name == "MANUAL":
				left_200_per, right_200_per = self.xy_mixing(self.str_percent, self.thr_percent)
				left_100_per = self.map(left_200_per, -200.0, 200.0, -100.0, 100.0)
				right_100_per = self.map(right_200_per, -200.0, 200.0, -100.0, 100.0)

			else:
				left_100_per = 0.0
				right_100_per = 0.0


			self.atcart_wheels_cmd_msg.data = [left_100_per, right_100_per]
			self.atcart_wheels_cmd_pub.publish(self.atcart_wheels_cmd_msg)


			print("mode: {:} | str: {:.1f} | thr: {:.1f} | vx: {:.1f} | wz: {:.1f} | left_100: {:.1f} | right_100: {:.1f}".format(\
				self.mode_name, self.str_percent, self.thr_percent, self.vx, self.wz, left_100_per, right_100_per))

			rate.sleep()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='ATCart sim for jmoab-ros')
	parser.add_argument('--ns',
						help="a namespace in front of topics")

	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	ns = args.ns

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	s = ATCartSim(ns)

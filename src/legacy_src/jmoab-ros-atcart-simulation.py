#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int8
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

		self.callback_timeout = 1.0 # second
		self.callback_timestamp = time.time()

		self.analog_steering = 1024
		self.analog_throttle = 1024

		self.sbus_mid = 1024
		self.sbus_min = 368
		self.sbus_max = 1680

		self.max_vel = 2.2	# m/s
		self.min_vel = -2.2

		self.max_ang_vel = 7.0	# rad/s
		self.min_ang_vel = -7.0

		self.cart_mode = 0
		self.mode_name = "HOLD"

		## -143	 RPM -->  368
		## 4.5   RPM -->  1100
		## 143   RPM -->  1680

		## Pub/Sub ##
		cmd_vel_topic = self.namespace_attaching(NS, "/cmd_vel")
		atcart_mode_topic = self.namespace_attaching(NS, "/atcart_mode")
		sbus_cmd_topic = self.namespace_attaching(NS, "/sbus_cmd")
		atcart_mode_cmd_topic = self.namespace_attaching(NS, "/atcart_mode_cmd")

		self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
		self.cmd_vel = Twist()

		self.atcart_mode_pub = rospy.Publisher(atcart_mode_topic, Int8, queue_size=10)
		self.atcart_mode = Int8()

		rospy.Subscriber(sbus_cmd_topic, Int32MultiArray, self.sbus_cmd_callback)
		rospy.Subscriber(atcart_mode_cmd_topic, Int8, self.cart_mode_callack)
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

	def sbus_cmd_callback(self, msg):

		self.cmd_steering = msg.data[0]
		self.cmd_throttle = msg.data[1]

		self.callback_timestamp = time.time()

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
		elif msg.buttons[1] == 1:
			self.cart_mode = 0
			self.mode_name = "HOLD"
		elif msg.buttons[2] == 1:
			self.cart_mode = 1
			self.mode_name = "MANUAL"

		self.analog_steering = int(self.map(msg.axes[3], -1.0, 1.0, self.sbus_max, self.sbus_min))
		self.analog_throttle = int(self.map(msg.axes[1], -1.0, 1.0, self.sbus_min, self.sbus_max))


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

	def sbus2vel(self,steering, throttle):
		if throttle != 1024:
			vx = self.map(throttle, self.sbus_min, self.sbus_max, self.min_vel, self.max_vel)
		else:
			vx = 0.0

		if vx >= 0:
			if steering != 1024:
				wz = self.map(steering, self.sbus_min, self.sbus_max, self.max_ang_vel, self.min_ang_vel)
			else:
				wz = 0.0
		else:
			if steering != 1024:
				wz = self.map(steering, self.sbus_min, self.sbus_max, self.min_ang_vel, self.max_ang_vel)
			else:
				wz = 0

		return vx, wz


	def loop(self):

		rate = rospy.Rate(20)

		sbus_steering  = 1024
		sbus_throttle = 1024

		while not rospy.is_shutdown():

			if self.mode_name == "AUTO":
				sbus_steering = self.cmd_steering
				sbus_throttle = self.cmd_throttle
			
			elif self.mode_name == "MANUAL":
				sbus_steering = self.analog_steering
				sbus_throttle = self.analog_throttle

			else:
				sbus_steering = 1024
				sbus_throttle = 1024


			if ((time.time() - self.callback_timestamp) > self.callback_timeout) and (self.mode_name == "AUTO"):
				vx = 0.0
				wz = 0.0
			else:
				vx, wz = self.sbus2vel(sbus_steering, sbus_throttle)


			self.cmd_vel.linear.x = vx
			self.cmd_vel.angular.z = wz
			self.cmd_vel_pub.publish(self.cmd_vel)

			self.atcart_mode = self.cart_mode
			self.atcart_mode_pub.publish(self.atcart_mode)

			# print("mode: {:} |str: {:d} | thr: {:d} | vx: {:.2f} | wz: {:.2f}".format(\
			# 	self.mode_name, sbus_steering, sbus_throttle, vx, wz))

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

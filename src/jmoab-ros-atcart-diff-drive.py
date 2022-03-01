#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray, Int8
import time
import argparse

class JMOAB_ATCart(object):

	def __init__(self, NS):

		rospy.init_node('jmoab_ros_atcart_diff_drive_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ATCart node")

		self.bus = SMBus(1)

		#### SBUS Steering Throttle ####
		self.sbus_steering_mid = 1024
		self.sbus_throttle_mid = 1024

		self.cmd_steering = self.sbus_steering_mid
		self.cmd_throttle = self.sbus_throttle_mid

		self.callback_timeout = 1.0 # second
		self.callback_timestamp = time.time()

		#### JMOAB I2C REG ####
		self.ATCART_MODE_REG = 0x52
		self.SBUS_FS_REG = 0x57

		self.sbus_max = 1680.0
		self.sbus_min = 368.0
		self.sbus_mid = 1024.0
		self.sbus_max_DB = self.sbus_mid + 1.0
		self.sbus_min_DB = self.sbus_mid - 1.0

		self.sbus_min_backward = 968	# a value before start rotating backward
		self.sbus_min_forward = 1093	# a value before start rotating forward
		self.prev_y = 0.0

		#### Pub/Sub ####
		sbus_rc_topic = self.namespace_attaching(NS, "/sbus_rc_ch")
		atcart_mode_topic = self.namespace_attaching(NS, "/atcart_mode")
		sbus_cmd_topic = self.namespace_attaching(NS, "/sbus_cmd")
		atcart_mode_cmd_topic = self.namespace_attaching(NS, "/atcart_mode_cmd")

		self.sbus_ch_pub = rospy.Publisher(sbus_rc_topic, Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()
		self.atcart_mode_pub = rospy.Publisher(atcart_mode_topic, Int8, queue_size=10)
		self.atcart_mode = Int8()

		rospy.Subscriber(sbus_cmd_topic, Int32MultiArray, self.cmd_callback)
		rospy.Subscriber(atcart_mode_cmd_topic, Int8, self.cart_mode_callack)


		rospy.loginfo("Publishing SBUS RC channel on {:} topic".format(sbus_rc_topic))
		rospy.loginfo("Subscribing on {:} topic for steering and throttle values".format(sbus_cmd_topic))
		rospy.loginfo("Publishing ATCart mode on {:} topic".format(atcart_mode_topic))
		rospy.loginfo("Subscribing on {:} topic for mode changing".format(atcart_mode_cmd_topic))

		## If want to bypass sbus failsafe uncomment this
		self.bypass_sbus_failsafe()

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


	def sbus2word(self, sbus_val):

		high_byte = sbus_val >> 8
		low_byte = (sbus_val & 0x00FF)

		return [high_byte, low_byte]

	def send_left_right(self, sbus_left, sbus_right):

		steering_bytes = self.sbus2word(sbus_left)
		throttle_bytes = self.sbus2word(sbus_right)

		## combine as 4 elements [str_H, str_L, thr_H, thr_L]
		all_bytes = steering_bytes+throttle_bytes

		self.bus.write_i2c_block_data(0x71, 0x30, all_bytes)

	def get_sbus_channel(self):
		input_SBUS = self.bus.read_i2c_block_data(0x71, 0x0C, 32)

		SBUS_ch = [None]*16
		for i in range(16):
			SBUS_ch[i] = (input_SBUS[(2*i)+1] & 0xFF) | ((input_SBUS[i*2] & 0xFF) << 8)

		return SBUS_ch

	def bypass_sbus_failsafe(self):
		## Disable SBUS Failsafe
		self.bus.write_byte_data(0x71, self.SBUS_FS_REG, 0x01)
		time.sleep(0.1)

		## Set back to hold mode
		self.write_atcart_mode(0x00)
		time.sleep(0.1)

		## Set back to auto mode
		self.write_atcart_mode(0x02)
		time.sleep(0.1)
		self.write_atcart_mode(0x02)
		time.sleep(0.1)
		self.write_atcart_mode(0x02)
		time.sleep(0.1)

	def read_atcart_mode(self):
		return self.bus.read_byte_data(0x71, self.ATCART_MODE_REG)

	def write_atcart_mode(self, mode_num):
		## need to write multiple times to take effect
		## publisher side only sends one time is ok
		for i in range(10):
			self.bus.write_byte_data(0x71, self.ATCART_MODE_REG, mode_num)

	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def cmd_callback(self, msg):
		if len(msg.data) > 0:
			cmd_steering = msg.data[0]
			cmd_throttle = msg.data[1]

			x_percent = self.sbus2percent(float(cmd_steering))
			y_percent = self.sbus2percent(float(cmd_throttle))

			left_percent, right_percent = self.xy_mixing(x_percent, y_percent)

			left_sbus, right_sbus = self.wheels_percent_to_wheels_sbus(left_percent, right_percent)

			self.send_left_right(int(left_sbus), int(right_sbus))
			self.callback_timestamp = time.time()
			
	def cart_mode_callack(self, msg):
		self.write_atcart_mode(msg.data)

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

	def sbus2percent(self, sbus):
		if sbus >= self.sbus_max_DB:
			percent = self.map(sbus, self.sbus_max_DB, self.sbus_max, 0.0, 100.0)
			if percent > 100.0:
				percent = 100.0
		elif sbus <= self.sbus_min_DB:
			percent = self.map(sbus, self.sbus_min, self.sbus_min_DB, -100.0, 0.0)
			if percent < -100.0:
				percent = -100.0
		else:
			percent = 0

		return percent

	def wheels_percent_to_wheels_sbus(self, left_per, right_per):

		if left_per > 0.0:
			left_sbus = self.map(left_per, 0.0, 200.0, self.sbus_min_forward, self.sbus_max)
		elif left_per < 0.0:
			left_sbus = self.map(left_per, -200.0, 0.0, self.sbus_min, self.sbus_min_backward)
		else:
			left_sbus = 1024

		if right_per > 0.0:
			right_sbus = self.map(right_per, 0.0, 200.0, self.sbus_min_forward, self.sbus_max)
		elif right_per < 0.0:
			right_sbus = self.map(right_per, -200.0, 0.0, self.sbus_min, self.sbus_min_backward)
		else:
			right_sbus = 1024

		return left_sbus, right_sbus


	def loop(self):

		rate = rospy.Rate(20) # 10hz

		while not rospy.is_shutdown():

			## Publish SBUS channel values
			sbus_ch_array = self.get_sbus_channel()
			self.sbus_ch.data = sbus_ch_array
			self.sbus_ch_pub.publish(self.sbus_ch)

			## if there is no sbus command until callback_timeout
			## then set back to neutral for all 
			if ((time.time() - self.callback_timestamp) > self.callback_timeout):
				self.send_left_right(int(self.sbus_mid), int(self.sbus_mid))

			## Get current atcart mode from JMOAB and publish to ros topic
			self.atcart_mode.data = self.read_atcart_mode()
			self.atcart_mode_pub.publish(self.atcart_mode)

			rate.sleep()


if __name__ == '__main__':

	parser = argparse.ArgumentParser(description='ATCart diff-drive for jmoab-ros')
	parser.add_argument('--ns',
						help="a namespace in front of topics")

	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	ns = args.ns

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")


	jmoab = JMOAB_ATCart(ns)
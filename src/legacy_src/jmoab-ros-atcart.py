#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray, Int8
import time

class JMOAB_ATCart:

	def __init__(self):

		rospy.init_node('jmoab_ros_atcart_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ATCart node")

		self.bus = SMBus(1)

		self.sbus_ch_pub = rospy.Publisher("/sbus_rc_ch", Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()

		self.atcart_mode_pub = rospy.Publisher("/atcart_mode", Int8, queue_size=10)
		self.atcart_mode = Int8()

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.cmd_callback)
		rospy.Subscriber("/atcart_mode_cmd", Int8, self.cart_mode_callack)

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


		rospy.loginfo("Publishing SBUS RC channel on /sbus_rc_ch topic")
		rospy.loginfo("Subscribing on /sbus_cmd topic for steering and throttle values")
		rospy.loginfo("Publishing ATCart mode on /atcart_mode topic")
		rospy.loginfo("Subscribing on /atcart_mode_cmd topic for mode changing")

		## If want to bypass sbus failsafe uncomment this
		self.bypass_sbus_failsafe()

		self.loop()

		rospy.spin()


	def sbus2word(self, sbus_val):

		high_byte = sbus_val >> 8
		low_byte = (sbus_val & 0x00FF)

		return [high_byte, low_byte]

	def send_steering_throttle(self, sbus_steering, sbus_throttle):

		steering_bytes = self.sbus2word(sbus_steering)
		throttle_bytes = self.sbus2word(sbus_throttle)

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


	def cmd_callback(self, msg):
		if len(msg.data) > 0:
			self.cmd_steering = msg.data[0]
			self.cmd_throttle = msg.data[1]
			self.send_steering_throttle(self.cmd_steering, self.cmd_throttle)
			self.callback_timestamp = time.time()
			
	def cart_mode_callack(self, msg):
		self.write_atcart_mode(msg.data)


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
				self.send_steering_throttle(self.sbus_steering_mid, self.sbus_throttle_mid)

			## Get current atcart mode from JMOAB and publish to ros topic
			self.atcart_mode.data = self.read_atcart_mode()
			self.atcart_mode_pub.publish(self.atcart_mode)

			rate.sleep()


if __name__ == '__main__':
	jmoab = JMOAB_ATCart()
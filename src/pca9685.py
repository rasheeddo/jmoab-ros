#! /usr/bin/env python
import rospy
import rospkg
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray, Int16MultiArray
import time
import struct
import numpy as np
import os
import argparse


class JMOAB_PCA9685(object):

	def __init__(self, addr, NS, NUM):

		rospy.init_node('jmoab_icm20948_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-PCA9685 node")

		rospy.loginfo("Subscribing on /pca9685_pwm topics")

		self.bus = SMBus(1)

		## PCA9685 address and registers
		if addr is None:
			self.ADDR = 0x40
		else:
			self.ADDR = int(addr, 0) # convert "0x29" a hex of string to integer

		self.total_servo = int(NUM)

		self.MODE1              = 0x00
		self.MODE2              = 0x01
		self.SUBADR1            = 0x02
		self.SUBADR2            = 0x03
		self.SUBADR3            = 0x04
		self.PRESCALE           = 0xFE
		self.LED0_ON_L          = 0x06
		self.LED0_ON_H          = 0x07
		self.LED0_OFF_L         = 0x08
		self.LED0_OFF_H         = 0x09
		self.ALL_LED_ON_L       = 0xFA
		self.ALL_LED_ON_H       = 0xFB
		self.ALL_LED_OFF_L      = 0xFC
		self.ALL_LED_OFF_H      = 0xFD

		self.RESTART            = 0x80
		self.SLEEP              = 0x10
		self.ALLCALL            = 0x01
		self.INVRT              = 0x10
		self.OUTDRV             = 0x04

		self.frequency = 50.0

		self.PWM_MIN = 720.0
		self.PWM_MAX = 2420.0
		self.PWM_MID = 1620.0 # for RC-ESC it seems to be 1620 of middle

		self.callback_timeout = 3.0
		self.callback_stamp = time.time()

		self.failsafe_enable = False

		self.init()
		self.set_pwm_freq(self.frequency)

		rospy.Subscriber(self.namespace_attaching(NS, "/pca9685_pwm"), Int16MultiArray, self.pwm_callback)
		
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

	def init(self):
		self.bus.write_byte_data(self.ADDR, self.MODE2, self.OUTDRV)
		self.bus.write_byte_data(self.ADDR, self.MODE1, self.ALLCALL)
		time.sleep(0.005)  # wait for oscillator
		mode1 = self.bus.read_byte_data(self.ADDR, self.MODE1)
		mode1 = mode1 & ~self.SLEEP  # wake up (reset sleep)
		self.bus.write_byte_data(self.ADDR, self.MODE1, mode1)
		time.sleep(0.005)  # wait for oscillator

		for i in range(self.total_servo):
			self.set_pwm(i, self.microsecToRegVal(self.PWM_MID))


	def set_pwm_freq(self, freq_hz):
		"""Set the PWM frequency to the provided value in hertz."""
		prescaleval = 25000000.0    # 25MHz
		prescaleval /= 4096.0       # 12-bit
		prescaleval /= float(freq_hz)
		prescaleval -= 1.0
		prescale = int(np.floor(prescaleval))
		oldmode = self.bus.read_byte_data(self.ADDR, self.MODE1);
		newmode = (oldmode & 0x7F) | 0x10    # sleep
		self.bus.write_byte_data(self.ADDR, self.MODE1, newmode)  # go to sleep
		self.bus.write_byte_data(self.ADDR, self.PRESCALE, prescale)
		self.bus.write_byte_data(self.ADDR, self.MODE1, oldmode)
		time.sleep(0.005)
		self.bus.write_byte_data(self.ADDR, self.MODE1, oldmode | 0x80)

	def set_pwm(self, channel, val):
		"""Sets a single PWM channel."""
		self.bus.write_byte_data(self.ADDR, self.LED0_ON_L+4*channel, 0 & 0xFF)
		self.bus.write_byte_data(self.ADDR, self.LED0_ON_H+4*channel, 0 >> 8)
		self.bus.write_byte_data(self.ADDR, self.LED0_OFF_L+4*channel, val & 0xFF)
		self.bus.write_byte_data(self.ADDR, self.LED0_OFF_H+4*channel, val >> 8)

	def microsecToRegVal(self, us):

		period_us = (1.0/self.frequency)*1E6

		val = int(us*4096.0/period_us)

		return val

	def over_limit_check(self, pwm):

		if pwm < self.PWM_MIN:
			pwm  = self.PWM_MIN
		elif pwm > self.PWM_MAX:
			pwm = self.PWM_MAX

		return pwm

	def pwm_callback(self, msg):

		if (len(msg.data) > 0) and (len(msg.data) <= 16):

			for i in range(len(msg.data)):
				val = self.over_limit_check(msg.data[i])
				self.set_pwm(i, self.microsecToRegVal(val))

		self.callback_stamp = time.time()



	def loop(self):
		rate = rospy.Rate(20) # 10hz

		while not rospy.is_shutdown():
			
			## if there is no data coming,
			if ((time.time() - self.callback_stamp) > self.callback_timeout) and self.failsafe_enable:
				for i in range(self.total_servo):
					self.set_pwm(i, self.microsecToRegVal(self.PWM_MID))

			rate.sleep()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='PCA9685 node')
	parser.add_argument('--addr',
						help="i2c address of the PCA9685, default is 0x40")
	parser.add_argument('--ns',
						help="a namespace in front of original topic")
	parser.add_argument('--num',
						help="a number of servo to use")


	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	addr = args.addr
	ns = args.ns
	num = args.num

	if addr is None:
		print("Using default i2c address as 0x40")

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	if num is None:
		print("Please provide at least --num 1 for one servo")
		quit()

	jmoab_pca9685 = JMOAB_PCA9685(addr, ns, num)
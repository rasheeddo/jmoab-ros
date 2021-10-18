#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray, Int8
import time
import RPi.GPIO as GPIO

class TMEJ_Motor:

	def __init__(self):

		rospy.init_node('jmoab_ros_dac_gpio_node', anonymous=True)
		rospy.loginfo("Start TMEJ Motor node")

		self.bus = SMBus(1)

		self.sbus_ch_pub = rospy.Publisher("/sbus_rc_ch", Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()

		self.atcart_mode_pub = rospy.Publisher("/atcart_mode", Int8, queue_size=10)
		self.atcart_mode = Int8()

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.cmd_callback)
		rospy.Subscriber("/atcart_mode_cmd", Int8, self.cart_mode_callback)

		## I2C Address ##
		self.DAC_L = 0x60
		self.DAC_R = 0x61
		self.WRITE_FAST_MODE = 0x00

		#### SBUS Steering Throttle ####
		self.sbus_steering_mid = 1024
		self.sbus_throttle_mid = 1024

		self.cmd_steering = self.sbus_steering_mid
		self.cmd_throttle = self.sbus_throttle_mid

		self.callback_timeout = 1.0 # second
		self.callback_timestamp = time.time()

		self.MAX_VOLT = 5.0
		self.MIN_VOLT = 0.0
		self.MAX_DEC = 3894
		self.MIN_DEC = 0

		self.cart_mode = 0

		self.sbus_stick_max = 1680.0
		self.sbus_stick_min = 368.0
		self.sbus_stick_mid = 1024.0
		self.sbus_max_DB = self.sbus_stick_mid + 10.0
		self.sbus_min_DB = self.sbus_stick_mid - 10.0

		self.y_percent = 0.0
		self.x_percent = 0.0

		self.left = 0.0
		self.right = 0.0

		self.prev_y = 0.0

		self.WHEEL_PERCENT_DB = 3.0
		self.volt_L = 0.0
		self.volt_R = 0.0
		self.dir_L = 0
		self.dir_R = 0

		self.force_auto = False

		self.GPIO13 = 33
		self.GPIO07 = 32
		self.GPIO11 = 31
		self.GPIO01 = 29

		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.GPIO13, GPIO.OUT, initial=GPIO.LOW)
		GPIO.setup(self.GPIO07, GPIO.OUT, initial=GPIO.LOW)
		GPIO.setup(self.GPIO11, GPIO.OUT, initial=GPIO.LOW)
		GPIO.setup(self.GPIO01, GPIO.OUT, initial=GPIO.LOW)


		#### JMOAB I2C REG ####
		self.ATCART_MODE_REG = 0x52
		self.SBUS_FS_REG = 0x57


		rospy.loginfo("Publishing SBUS RC channel on /sbus_rc_ch topic")
		rospy.loginfo("Subscribing on /sbus_cmd topic for steering and throttle values")
		rospy.loginfo("Publishing cart mode on /atcart_mode topic")
		rospy.loginfo("Subscribing on /atcart_mode_cmd topic for mode changing")

		self.write_i2c(self.DAC_L, self.volt_L)
		self.write_i2c(self.DAC_R, self.volt_R)


		self.loop()

		rospy.spin()


	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def volt2word(self, volt):

		raw_dec_12bit = int(self.map(volt, self.MIN_VOLT, self.MAX_VOLT, self.MIN_DEC, self.MAX_DEC))

		raw_dec_12bit &= 0xFFF 
		data1 = self.WRITE_FAST_MODE | raw_dec_12bit >> 8
		data2 = raw_dec_12bit & 0x0FF

		return [data1, data2]

	def get_sbus_channel(self):
		input_SBUS = self.bus.read_i2c_block_data(0x71, 0x0C, 32)

		SBUS_ch = [None]*16
		for i in range(16):
			SBUS_ch[i] = (input_SBUS[(2*i)+1] & 0xFF) | ((input_SBUS[i*2] & 0xFF) << 8)

		return SBUS_ch

	def cmd_callback(self, msg):
		if len(msg.data) > 0:
			self.cmd_steering = msg.data[0]
			self.cmd_throttle = msg.data[1]
			self.callback_timestamp = time.time()

	def cart_mode_callback(self,msg):

		if msg.data == 2:
			self.force_auto = True
			# print(self.force_auto)
		else:
			self.force_auto = False

	def sbus2percent(self, sbus):
		if sbus >= self.sbus_max_DB:
			percent = self.map(sbus, self.sbus_max_DB, self.sbus_stick_max, 0.0, 100.0)
			if percent > 100.0:
				percent = 100.0
		elif sbus <= self.sbus_min_DB:
			percent = self.map(sbus, self.sbus_stick_min, self.sbus_min_DB, -100.0, 0.0)
			if percent < -100.0:
				percent = -100.0
		else:
			percent = 0

		return percent

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

	def wheel_percent_to_voltage_dir(self, wheel_per):

		## take a wheel of -200 to 200 range percent 
		## and convert to voltage of 0-5V and direction
		if wheel_per > self.WHEEL_PERCENT_DB:
			volt = abs(self.map(wheel_per, 0.0, 200.0, 0.0, 5.0))
			_dir = 1

		elif wheel_per < -self.WHEEL_PERCENT_DB:
			volt = abs(self.map(wheel_per, 0.0, 200.0, 0.0, 5.0))
			_dir = -1
		else:
			volt = 0.0
			_dir = 0

		return volt, _dir

	def write_i2c(self, i2c_addr, volt):

		data_bytes = self.volt2word(volt)
		self.bus.write_block_data(i2c_addr, 0x00, data_bytes)

	def write_gpio(self, dir_L, dir_R):

		if (dir_L == 0):
			GPIO.output(self.GPIO11, GPIO.LOW)
			GPIO.output(self.GPIO01, GPIO.LOW)
		elif (dir_L == 1):
			GPIO.output(self.GPIO11, GPIO.LOW)
			GPIO.output(self.GPIO01, GPIO.HIGH)
		elif (dir_L == -1):
			GPIO.output(self.GPIO11, GPIO.HIGH)
			GPIO.output(self.GPIO01, GPIO.LOW)
		else:
			GPIO.output(self.GPIO11, GPIO.LOW)
			GPIO.output(self.GPIO01, GPIO.LOW)

		if (dir_R == 0):
			GPIO.output(self.GPIO13, GPIO.LOW)
			GPIO.output(self.GPIO07, GPIO.LOW)
		elif (dir_R == 1):
			GPIO.output(self.GPIO13, GPIO.LOW)
			GPIO.output(self.GPIO07, GPIO.HIGH)
		elif (dir_R == -1):
			GPIO.output(self.GPIO13, GPIO.HIGH)
			GPIO.output(self.GPIO07, GPIO.LOW)
		else:
			GPIO.output(self.GPIO13, GPIO.LOW)
			GPIO.output(self.GPIO07, GPIO.LOW)



	def loop(self):

		rate = rospy.Rate(10) # 10hz
		mode_str = "hold"
		prev_ch5 = 1024

		while not rospy.is_shutdown():

			## Publish SBUS channel values
			sbus_ch_array = self.get_sbus_channel()
			self.sbus_ch.data = sbus_ch_array
			self.sbus_ch_pub.publish(self.sbus_ch)

			## Reset force_auto if there is some change on ch5 switch
			if sbus_ch_array[4] != prev_ch5:
				self.force_auto = False

			## Auto mode, we could change to auto mode by sbus_ch5 and atcart_mode_cmd
			if (sbus_ch_array[4] > 1500) or self.force_auto:
				self.x_percent = self.sbus2percent(self.cmd_steering)
				self.y_percent = self.sbus2percent(self.cmd_throttle)
				mode_str = "auto"

			## Manual mode
			elif sbus_ch_array[4] == 1024:
				self.x_percent = self.sbus2percent(sbus_ch_array[0])
				self.y_percent = self.sbus2percent(sbus_ch_array[1])
				mode_str = "manual" 

			## HOLD mode
			elif sbus_ch_array[4] < 900:
				self.x_percent = 0.0
				self.y_percent = 0.0
				mode_str = "hold"

			## from x,y percentages for stick motion, we convert to left/right percentage as -200,200 ranges
			self.left, self.right = self.xy_mixing(self.x_percent, self.y_percent)
			## from left/right percentages, we convert to voltage and direction sign
			self.volt_L, self.dir_L = self.wheel_percent_to_voltage_dir(self.left)
			self.volt_R, self.dir_R = self.wheel_percent_to_voltage_dir(self.right)
			## write voltage/dir to i2c device and gpio
			self.write_i2c(self.DAC_L, self.volt_L)
			self.write_i2c(self.DAC_R, self.volt_R)
			self.write_gpio(self.dir_L, self.dir_R)

			# if there is no sbus command until callback_timeout
			# then set back to neutral for all 
			if ((time.time() - self.callback_timestamp) > self.callback_timeout):
				self.cmd_steering = 1024
				self.cmd_throttle = 1024

			## Get current atcart mode from JMOAB and publish to ros topic
			if sbus_ch_array[4] == 144:
				self.cart_mode = 0
			elif sbus_ch_array == 1024:
				self.cart_mode = 1
			elif sbus_ch_array == 1904:
				self.cart_mode = 2

			self.atcart_mode.data = self.cart_mode
			self.atcart_mode_pub.publish(self.atcart_mode)

			print("mode: {:} | L: {:.2f} | R: {:.2f} | volt_L: {:.3f} | dir_L: {:d} | volt_R: {:.3f} | dir_R: {:d}".format(\
			mode_str, self.left, self.right, self.volt_L, self.dir_L, self.volt_R, self.dir_R)) 

			prev_ch5 = sbus_ch_array[4]

			rate.sleep()


if __name__ == '__main__':
	jmoab = TMEJ_Motor()
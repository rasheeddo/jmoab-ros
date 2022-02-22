#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray, Int8
import time

class JMOAB_PWMCart:

	def __init__(self):

		rospy.init_node('jmoab_ros_atcart_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-PWMCart node")

		self.bus = SMBus(1)

		self.sbus_ch_pub = rospy.Publisher("/sbus_rc_ch", Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()

		self.atcart_mode_pub = rospy.Publisher("/atcart_mode", Int8, queue_size=10)
		self.atcart_mode = Int8()

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.sbus_cmd_callback)
		rospy.Subscriber("/atcart_mode_cmd", Int8, self.cart_mode_callack)

		self.sbus_min = 368
		self.sbus_max = 1680
		self.sbus_mid = 1024

		self.pwm_min = 1120
		self.pwm_max = 1920
		self.pwm_mid = 1520

		self.prev_Y = 0.0

		self.pwm_steering_mid = 1520
		self.pwm_throttle_mid = 1520

		self.cmd_steering = self.sbus_mid
		self.cmd_throttle = self.sbus_mid

		self.callback_timeout = 5.0 # second
		self.callback_timestamp = time.time()

		self.rev_str = True #False
		self.rev_thr = True

		#### JMOAB I2C REG ####
		self.ATCART_MODE_REG = 0x52
		self.SBUS_FS_REG = 0x57

		self.mode = "AUTO"
		self.mode_num = 2


		rospy.loginfo("Publishing SBUS RC channel on /sbus_rc_ch topic")
		rospy.loginfo("Subscribing on /sbus_cmd topic for steering and throttle values")
		rospy.loginfo("Publishing ATCart mode on /atcart_mode topic")
		rospy.loginfo("Subscribing on /atcart_mode_cmd topic for mode changing")

		self.bypass_sbus_failsafe()
		
		self.loop()

		rospy.spin()

	def bypass_sbus_failsafe(self):
		## Disable SBUS Failsafe
		self.bus.write_byte_data(0x71, self.SBUS_FS_REG, 0x01)
		time.sleep(0.1)

		## Set back to hold mode
		self.write_atcart_mode(0x00)
		time.sleep(0.1)

		## Set back to manual mode
		self.write_atcart_mode(0x01)
		time.sleep(0.1)
		self.write_atcart_mode(0x01)
		time.sleep(0.1)
		self.write_atcart_mode(0x01)
		time.sleep(0.1)

	def write_atcart_mode(self, mode_num):
		## need to write multiple times to take effect
		## publisher side only sends one time is ok
		for i in range(10):
			self.bus.write_byte_data(0x71, self.ATCART_MODE_REG, mode_num)

	def read_atcart_mode(self):
		self.mode_num = self.bus.read_byte_data(0x71, self.ATCART_MODE_REG)
		if self.mode_num == 0:
			self.mode = "HOLD"
		elif self.mode_num == 1:
			self.mode = "MANUAL"
		elif self.mode_num == 2:
			self.mode = "AUTO"
		else:
			self.mode = "UNKNOWN"

		return self.mode_num 

	def cart_mode_callack(self, msg):
		self.write_atcart_mode(msg.data)
		# if msg.data == 0:
		# 	self.mode = "HOLD"
		# 	self.mode_num = 0
		# elif msg.data == 1:
		# 	self.mode = "MANUAL"
		# 	self.mode_num = 1
		# elif msg.data == 2:
		# 	self.mode = "AUTO"
		# 	self.mode_num = 2
		# else:
		# 	self.mode = "UNKNOWN"
		# 	self.mode_num = 3

	def pwm2word(self, pwm_val):

		high_byte = pwm_val >> 8
		low_byte = (pwm_val & 0x00FF)

		return [high_byte, low_byte]

	def send_pwm_leftright(self, pwm_left, pwm_right):

		left_bytes = self.pwm2word(pwm_left)
		right_bytes = self.pwm2word(pwm_right)

		## combine as 4 elements [str_H, str_L, thr_H, thr_L]
		all_bytes = left_bytes+right_bytes

		self.bus.write_i2c_block_data(0x71, 0x06, all_bytes)

	def get_sbus_channel(self):
		input_SBUS = self.bus.read_i2c_block_data(0x71, 0x0C, 32)

		SBUS_ch = [None]*16
		for i in range(16):
			SBUS_ch[i] = (input_SBUS[(2*i)+1] & 0xFF) | ((input_SBUS[i*2] & 0xFF) << 8)

		return SBUS_ch

	def sbus_cmd_callback(self, msg):
		if len(msg.data) > 0:
			self.cmd_steering = msg.data[0]
			self.cmd_throttle = msg.data[1]
			# self.send_steering_throttle(self.cmd_steering, self.cmd_throttle)
			self.callback_timestamp = time.time()

	def map(self, val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def channel_mixing(self, str_ch, thr_ch):

		if self.rev_thr:
			y = self.map(thr_ch, self.sbus_min, self.sbus_max, 100.0, -100.0)
		else:
			y = self.map(thr_ch, self.sbus_min, self.sbus_max, -100.0, 100.0)

		if self.rev_str:
			x = self.map(str_ch, self.sbus_min, self.sbus_max, 100.0, -100.0)
		else:
			x = self.map(str_ch, self.sbus_min, self.sbus_max, -100.0, 100.0)

		left = y+x
		right = y-x

		diff = abs(x) - abs(y)

		if left < 0.0:
			left = left - abs(diff)
		else:
			left = left + abs(diff)

		if right < 0.0:
			right = right - abs(diff)
		else:
			right = right + abs(diff)


		# if (self.prev_Y < 0.0):
		# 	swap = left
		# 	left = right
		# 	right = swap

		self.prev_Y = y

		left_pwm = self.map(left, -200.0, 200.0, self.pwm_min, self.pwm_max)
		right_pwm = self.map(right, -200.0, 200.0, self.pwm_min, self.pwm_max)

		return int(left_pwm), int(right_pwm)


	def loop(self):

		rate = rospy.Rate(20) # 10hz
		prev_ch5 = 1024

		while not rospy.is_shutdown():

			sbus_ch_array = self.get_sbus_channel()

			self.sbus_ch.data = sbus_ch_array
			self.sbus_ch_pub.publish(self.sbus_ch)

			# if sbus_ch_array[4] > 1500 or ((self.mode == "AUTO") and not propo_mode_changed):
			if self.mode_num == 2:
				if ((time.time() - self.callback_timestamp) > self.callback_timeout):
					left_pwm = 1520
					right_pwm = 1520
					self.cmd_steering = 1024
					self.cmd_throttle = 1024
					print("SBUS Timeout...")
				else:
					left_pwm, right_pwm = self.channel_mixing(self.cmd_steering, self.cmd_throttle)
					
				self.mode_num = 2
				self.mode = "AUTO"

			elif self.mode_num == 1:
			# elif sbus_ch_array[4] < 1500:
				left_pwm, right_pwm = self.channel_mixing(sbus_ch_array[0], sbus_ch_array[1])
				self.mode = "MANUAL"
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
				self.mode_num = 1

			elif self.mode_num == 0:
			# elif sbus_ch_array[4] < 700 or ((self.mode == "HOLD") and not propo_mode_changed):
				left_pwm = self.pwm_mid
				right_pwm = self.pwm_mid
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
				self.mode = "HOLD"
				self.mode_num = 0

			else:
				self.mode = "UNKNOWN"
				left_pwm = 1520
				right_pwm = 1520
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
				self.mode_num = 3

			## Set limit for safety
			if (900 < left_pwm < 2100) and (900 < right_pwm < 2100): 
				self.send_pwm_leftright(left_pwm, right_pwm)


			## Logging to screen
			# if self.mode != "AUTO":
			# 	print("mode: {:} | mode_num: {:d} | sbus_str: {:d} | sbus_thr: {:d} | pwm_left: {:d} | pwm_right: {:d}".format(\
			# 		self.mode, self.mode_num, sbus_ch_array[0], sbus_ch_array[1], left_pwm, right_pwm))
			# else:
			# 	print("mode: {:} | mode_num: {:d} | sbus_str: {:d} | sbus_thr: {:d} | pwm_left: {:d} | pwm_right: {:d}".format(\
			# 		self.mode, self.mode_num, self.cmd_steering, self.cmd_throttle, left_pwm, right_pwm))


			self.atcart_mode.data =  self.read_atcart_mode() #self.mode_num
			self.atcart_mode_pub.publish(self.atcart_mode)


			rate.sleep()


if __name__ == '__main__':
	jmoab = JMOAB_PWMCart()
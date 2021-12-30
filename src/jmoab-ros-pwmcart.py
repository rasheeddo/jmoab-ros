#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray
import time

class JMOAB_PWMCart:

	def __init__(self):

		rospy.init_node('jmoab_ros_atcart_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-PWMCart node")

		self.bus = SMBus(1)

		self.sbus_ch_pub = rospy.Publisher("/sbus_rc_ch", Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.sbus_cmd_callback)

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

		self.callback_timeout = 1.0 # second
		self.callback_timestamp = time.time()

		self.rev_str = True #False
		self.rev_thr = True


		rospy.loginfo("Publishing SBUS RC channel on /sbus_rc_ch topic")
		rospy.loginfo("Subscribing on /sbus_cmd topic for steering and throttle values")

		self.loop()

		rospy.spin()


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

		while not rospy.is_shutdown():

			sbus_ch_array = self.get_sbus_channel()
			self.sbus_ch.data = sbus_ch_array
			self.sbus_ch_pub.publish(self.sbus_ch)

			if sbus_ch_array[4] < 700:
				left_pwm = self.pwm_mid
				right_pwm = self.pwm_mid
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
				mode = "HOLD"
			elif sbus_ch_array[4] < 1500:
				left_pwm, right_pwm = self.channel_mixing(sbus_ch_array[0], sbus_ch_array[1])
				mode = "MANUAL"
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
			else:
				if ((time.time() - self.callback_timestamp) > self.callback_timeout):
					mode = "AUTO"
					left_pwm = 1520
					right_pwm = 1520
				else:
					left_pwm, right_pwm = self.channel_mixing(self.cmd_steering, self.cmd_throttle)
					mode = "AUTO"

			self.send_pwm_leftright(left_pwm, right_pwm)


			if mode != "AUTO":
				print("mode: {:} | sbus_str: {:d} | sbus_thr: {:d} | pwm_left: {:d} | pwm_right: {:d}".format(\
					mode, sbus_ch_array[0], sbus_ch_array[1], left_pwm, right_pwm))
			else:
				print("mode: {:} | sbus_str: {:d} | sbus_thr: {:d} | pwm_left: {:d} | pwm_right: {:d}".format(\
					mode, self.cmd_steering, self.cmd_throttle, left_pwm, right_pwm))

			## if there is no sbus command until callback_timeout
			## then set back to neutral for all 
			# if ((time.time() - self.callback_timestamp) > self.callback_timeout):
			# 	self.send_steering_throttle(self.sbus_steering_mid, self.sbus_throttle_mid)

			rate.sleep()


if __name__ == '__main__':
	jmoab = JMOAB_PWMCart()
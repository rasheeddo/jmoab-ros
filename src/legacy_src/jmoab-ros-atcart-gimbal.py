#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray, Bool
import time

class JMOAB_ATCart:

	def __init__(self):

		rospy.init_node('jmoab_ros_atcart_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ATCart-RONIN-Gimbal node")

		self.bus = SMBus(1)

		self.sbus_ch_pub = rospy.Publisher("/sbus_rc_ch", Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.cmd_callback)
		rospy.Subscriber("/sbus_gimbal_cmd", Int32MultiArray, self.gimbal_cmd_callback)
		rospy.Subscriber("/gimbal_recenter", Bool, self.gimbal_recenter_callback)

		#####################
		### I2C Registers ###
		#####################
		self.GIMBAL_PAN_H_REG = 0x30
		self.GIMBAL_PAN_L_REG = 0x31
		self.GIMBAL_TIL_H_REG = 0x32
		self.GIMBAL_TIL_L_REG = 0x33
		self.GIMBAL_CEN_H_REG = 0x3c
		self.GIMBAL_CEN_L_REG = 0x3d

		self.SBUS_IN_CH1_H = 0x0c

		self.CARTIF_STR_H_REG = 0x40
		self.CARTIF_STR_L_REG = 0x41
		self.CARTIF_THR_H_REG = 0x42
		self.CARTIF_THR_L_REG = 0x43
		self.CARTIF_MODE_REG = 0x52
		self.CARTIF_SBUS_FS = 0x57

		###################
		### SBUS values ###
		###################
		self.sbus_mid = 1024
		self.sbus_min = 144
		self.sbus_max = 1904
		
		#### Cart ####
		self.sbus_steering_mid = self.sbus_mid
		self.sbus_throttle_mid = self.sbus_mid

		self.cmd_steering = self.sbus_steering_mid
		self.cmd_throttle = self.sbus_throttle_mid

		self.last_ch5 = self.sbus_mid

		#### Gimbal ####
		self.sbus_cmd_pan = self.sbus_mid
		self.sbus_cmd_tilt = self.sbus_mid

		self.gimbal_recenter_flag = True

		###############
		### Timeout ###
		###############
		self.callback_timeout = 1.0 # second
		self.callback_timestamp = time.time()

		self.gimbal_callback_timout = 1.0
		self.gimbal_callback_timestamp = time.time()


		rospy.loginfo("Publishing SBUS RC channel on /sbus_rc_ch topic")
		rospy.loginfo("Subscribing on /sbus_cmd topic for steering and throttle values")

		## If want to bypass sbus failsafe uncomment this
		#self.bypass_sbus_failsafe()

		self.recenter_gimbal()

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

		self.bus.write_i2c_block_data(0x71, self.CARTIF_STR_H_REG, all_bytes)

	def get_sbus_channel(self):
		input_SBUS = self.bus.read_i2c_block_data(0x71, self.SBUS_IN_CH1_H, 32)

		SBUS_ch = [None]*16
		for i in range(16):
			SBUS_ch[i] = (input_SBUS[(2*i)+1] & 0xFF) | ((input_SBUS[i*2] & 0xFF) << 8)

		return SBUS_ch

	def bypass_sbus_failsafe(self):
		## Disable SBUS Failsafe
		self.bus.write_byte_data(0x71, self.CARTIF_SBUS_FS, 0x01)
		time.sleep(0.1)

		## Set back to hold mode
		self.bus.write_byte_data(0x71, self.CARTIF_MODE_REG, 0x00)
		time.sleep(0.1)

		## Set back to auto mode
		self.bus.write_byte_data(0x71, self.CARTIF_MODE_REG, 0x02)
		time.sleep(0.1)

	def change_cart_mode(self, mode):

		self.bus.write_byte_data(0x71, self.CARTIF_MODE_REG, mode)

	def send_pan_tilt(self, sbus_pan, sbus_tilt):

		pan_bytes = self.sbus2word(sbus_pan)
		tilt_bytes = self.sbus2word(sbus_tilt)

		all_bytes = pan_bytes+tilt_bytes

		self.bus.write_i2c_block_data(0x71, self.GIMBAL_PAN_H_REG, all_bytes)

	def recenter_gimbal(self):

		self.bus.write_i2c_block_data(0x71, self.GIMBAL_CEN_H_REG, self.sbus2word(144))
		time.sleep(0.2)	# this delay comes from experiment, MUST NOT CHANGE
		self.bus.write_i2c_block_data(0x71, self.GIMBAL_CEN_H_REG, self.sbus2word(1024))

		self.gimbal_recenter_flag = False

	def cmd_callback(self, msg):
		if len(msg.data) > 0:
			self.cmd_steering = msg.data[0]
			self.cmd_throttle = msg.data[1]
			self.callback_timestamp = time.time()

	def gimbal_cmd_callback(self, msg):

		if len(msg.data) > 0:

			self.sbus_cmd_pan = msg.data[0]
			self.sbus_cmd_tilt = msg.data[1]

			self.gimbal_callback_timestamp = time.time()

	def gimbal_recenter_callback(self, msg):

		self.gimbal_recenter_flag = msg.data

	def loop(self):

		rate = rospy.Rate(20) # 10hz

		while not rospy.is_shutdown():

			sbus_ch_array = self.get_sbus_channel()
			self.sbus_ch.data = sbus_ch_array
			self.sbus_ch_pub.publish(self.sbus_ch)


			## Check ch5 is changed or not
			if sbus_ch_array[4] != self.last_ch5:
				if sbus_ch_array[4] > 1600:
					self.change_cart_mode(2)
				elif sbus_ch_array[4] > 900:
					self.change_cart_mode(1)
				else:
					self.change_cart_mode(0)

				self.send_steering_throttle(self.sbus_steering_mid, self.sbus_throttle_mid)

 
			## Check ch5 is auto and timeout then control cart
			if sbus_ch_array[4] > 1600:
				if ((time.time() - self.callback_timestamp) > self.callback_timeout):
					self.send_steering_throttle(self.sbus_steering_mid, self.sbus_throttle_mid)
				else:
					self.send_steering_throttle(self.cmd_steering, self.cmd_throttle)


			## Check gimbal timeout then control gimbal
			if ((time.time() - self.gimbal_callback_timestamp) > self.gimbal_callback_timout):
				self.send_pan_tilt(self.sbus_mid, self.sbus_mid)
			else:
				self.send_pan_tilt(self.sbus_cmd_pan, self.sbus_cmd_tilt)

			if self.gimbal_recenter_flag:
				self.recenter_gimbal()


			self.last_ch5 = sbus_ch_array[4]

			rate.sleep()


if __name__ == '__main__':
	jmoab = JMOAB_ATCart()
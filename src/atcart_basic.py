#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int16MultiArray, UInt8, Int8MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist
import time
import argparse
import numpy as np

JMOAB_I2C_ADDR1 = 0x70
JMOAB_I2C_ADDR2 = 0x71

## address1 registers
RY1 = 0x00
RY2 = 0x01

## address2
ADC_A0 = 0x00
PWM1_H = 0x06
PWM2_H = 0x08
PWM3_H = 0x0A
IN_SBUS_CH1_H = 0x0C
CARTIF_SBUS_CH1_H = 0x30
CARTIF_INPUT_SELECT = 0x52
AUX_LED1 = 0x54
DISABLE_SBUS_FAILSAFE = 0x57

HOLD = 0x00
MAN = 0x01
AUTO = 0x02

ENA_FS = 0x00
DIS_FS = 0x01

class JMOAB_ATCart(object):

	def __init__(self, NS):

		rospy.init_node('jmoab_ros_atcart_basic_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ATCart node")

		self.bus = SMBus(1)

		#### SBUS Steering Throttle ####
		self.sbus_steering_mid = 1024
		self.sbus_throttle_mid = 1024

		self.cmd_steering = self.sbus_steering_mid
		self.cmd_throttle = self.sbus_throttle_mid

		self.sbus_max = 1680.0
		self.sbus_min = 368.0
		self.sbus_mid = 1024.0
		self.sbus_max_DB = self.sbus_mid + 1.0
		self.sbus_min_DB = self.sbus_mid - 1.0

		self.sbus_min_backward = 968	# a value before start rotating backward
		self.sbus_min_forward = 1093	# a value before start rotating forward
		self.prev_y = 0.0

		### cmd_vel
		self.vx_max = 2.0
		self.wz_max = 2.0
		self.vx = 0.0
		self.wz = 0.0
		self.cmd_vel_cb_flag = False
		self.cmd_vel_timeout = 1.0
		self.cmd_vel_stamp = time.time()

		### wheels_cmd
		self.left_percent = 0.0
		self.right_percent = 0.0 
		self.wheels_cmd_cb_flag = False
		self.wheels_cmd_timeout = 1.0
		self.wheels_cmd_stamp = time.time()

		### cart_mode_cmd
		self.cart_mode_cmd = 1
		self.cart_mode_cb_flag = False

		### relay cmd
		self.relay_list = []
		self.relay_cb_flag = False

		### servo cmd
		self.pwm_min = 1120
		self.pwm_max = 1920
		self.pwm_mid = 1520
		self.servo_list = []
		self.servo_cb_flag = False

		#### Pub/Sub ####
		sbus_rc_topic = self.namespace_attaching(NS, "/jmoab/sbus_rc_ch")
		cart_mode_topic = self.namespace_attaching(NS, "/jmoab/cart_mode")
		wheels_cmd_topic = self.namespace_attaching(NS, "/jmoab/wheels_cmd")
		cart_mode_cmd_topic = self.namespace_attaching(NS, "/jmoab/cart_mode_cmd")
		relay_topic = self.namespace_attaching(NS, "/jmoab/relays")
		adc_topic = self.namespace_attaching(NS, '/jmoab/adc')
		servo_topic = self.namespace_attaching(NS, '/jmoab/servos')

		cmd_vel_topic = self.namespace_attaching(NS, "/cmd_vel")

		self.sbus_ch_pub = rospy.Publisher(sbus_rc_topic, Int16MultiArray, queue_size=1)
		self.cart_mode_pub = rospy.Publisher(cart_mode_topic, UInt8, queue_size=1)
		self.adc_pub = rospy.Publisher(adc_topic, Float32MultiArray, queue_size=1)

		rospy.Subscriber(wheels_cmd_topic, Float32MultiArray, self.wheels_cmd_callback)
		rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)
		rospy.Subscriber(cart_mode_cmd_topic, UInt8, self.cart_mode_callack)
		rospy.Subscriber(relay_topic, Int8MultiArray, self.relay_callback)
		rospy.Subscriber(servo_topic, Int16MultiArray, self.servo_callback)

		rospy.loginfo("Namespace is {}".format(NS))
		rospy.loginfo("Publishing to  {:} topic as std_msgs/Int16MultiArray".format(sbus_rc_topic))
		rospy.loginfo("Publishing to  {:} topic as std_msgs/UInt8".format(cart_mode_topic))
		rospy.loginfo("Publishing to  {:} topic as std_msgs/Float32MultiArray".format(adc_topic))

		rospy.loginfo("Subscribing on {:} topic as std_msgs/Float32MultiArray  ex: [30.0, -50.0] percentages -100.0 to 100.0 ranges".format(wheels_cmd_topic))
		rospy.loginfo("Subscribing on {:} topic as geometry_msgs/Twist  ex: linear.x angular.z".format(cmd_vel_topic))
		rospy.loginfo("Subscribing on {:} topic as std_msgs/UInt8  ex: 0=hold, 1=manual, 2=auto".format(cart_mode_cmd_topic))
		rospy.loginfo("Subscribing on {:} topic as std_msgs/Int8MultiArray  ex: [1,1] on both relays".format(relay_topic))
		rospy.loginfo("Subscribing on {:} topic as std_msgs/Int16MultiArray  ex: [19200, 1120, 1520] ".format(servo_topic))

		## If want to bypass sbus failsafe uncomment this
		#self.bypass_sbus_failsafe()

		self.loop()

		rospy.spin()

	##################################
	### ROS and callback functions ###
	##################################
	def namespace_attaching(self, NS, topic_name):
		if NS is None:
			return topic_name
		else:
			if NS.startswith("/"):
				topic_name = NS + topic_name
			else:
				topic_name = "/" + NS + topic_name
			return topic_name

	# def cmd_callback(self, msg):

		# if len(msg.data) > 0:
		# 	cmd_steering = msg.data[0]
		# 	cmd_throttle = msg.data[1]

		# 	x_percent = self.sbus2percent(float(cmd_steering))
		# 	y_percent = self.sbus2percent(float(cmd_throttle))

		# 	left_percent, right_percent = self.xy_mixing(x_percent, y_percent)

		# 	left_sbus, right_sbus = self.wheels_percent_to_wheels_sbus(left_percent, right_percent)

		# 	self.send_left_right(int(left_sbus), int(right_sbus))
		# 	self.callback_timestamp = time.time()

	def cart_mode_callack(self, msg):
		self.cart_mode_cb_flag = True
		self.cart_mode_cmd = msg.data

	def relay_callback(self, msg):

		self.relay_list = msg.data

		self.relay_cb_flag = True

	def servo_callback(self, msg):
		self.servo_cb_flag = True
		self.servo_list = msg.data

	def cmd_vel_callback(self, msg):

		if msg.linear.x > self.vx_max:
			vx = self.vx_max
		elif msg.linear.x < -self.vx_max:
			vx = -self.vx_max
		else:
			vx = msg.linear.x

		if msg.angular.z > self.wz_max:
			wz = self.wz_max
		elif msg.angular.z < -self.wz_max:
			wz = -self.wz_max
		else:
			wz = msg.angular.z

		self.vx = vx
		self.wz = wz

		self.cmd_vel_stamp = time.time()
		self.cmd_vel_cb_flag = True

	def wheels_cmd_callback(self, msg):

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

		self.left_percent = left
		self.right_percent = right

		self.wheels_cmd_cb_flag = True
		self.wheels_cmd_stamp = time.time()



	##################################
	### SMBus read/write functions ###
	##################################

	def bypass_sbus_failsafe(self):
		## Disable SBUS Failsafe
		self.bus.write_byte_data(JMOAB_I2C_ADDR2, DISABLE_SBUS_FAILSAFE, DIS_FS)
		time.sleep(0.1)

		## Set back to hold mode
		self.write_atcart_mode(HOLD)
		time.sleep(0.1)

		## Set back to auto mode
		self.write_atcart_mode(AUTO)
		time.sleep(0.1)
		self.write_atcart_mode(AUTO)
		time.sleep(0.1)
		self.write_atcart_mode(AUTO)
		time.sleep(0.1)

	def read_sbus_ch(self):
		input_SBUS = self.bus.read_i2c_block_data(JMOAB_I2C_ADDR2, IN_SBUS_CH1_H, 20)

		SBUS_ch = [None]*10
		for i in range(10):
			SBUS_ch[i] = (input_SBUS[(2*i)+1] & 0xFF) | ((input_SBUS[i*2] & 0xFF) << 8)

		return SBUS_ch

	def read_atcart_mode(self):
		return self.bus.read_byte_data(JMOAB_I2C_ADDR2, CARTIF_INPUT_SELECT)

	def read_adc(self):
		raw = self.bus.read_i2c_block_data(JMOAB_I2C_ADDR2, ADC_A0, 6)
		voltage_list = self.convert2voltage(raw)
		return voltage_list

	def write_left_right(self, sbus_left, sbus_right):

		steering_bytes = self.sbus2word(sbus_left)
		throttle_bytes = self.sbus2word(sbus_right)

		## combine as 4 elements [str_H, str_L, thr_H, thr_L]
		all_bytes = steering_bytes+throttle_bytes

		self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, CARTIF_SBUS_CH1_H, all_bytes)

	def write_atcart_mode(self, mode_num):
		## need to write multiple times to take effect
		## publisher side only sends one time is ok
		for i in range(10):
			self.bus.write_byte_data(JMOAB_I2C_ADDR2, CARTIF_INPUT_SELECT, mode_num)

	def write_relay(self, relay_list):
		all_bytes = [int(relay_list[1]), int(relay_list[0])]
		self.bus.write_i2c_block_data(JMOAB_I2C_ADDR1, RY1, all_bytes)

	def write_servo(self, servo_list):

		if len(servo_list) > 0 and len(servo_list) <= 3:

			for servo_id in range(len(servo_list)):
				cmd_pwm = servo_list[servo_id]
				cmd_pwm = self.over_limit_check(cmd_pwm)
				self.write_pwm_bytes(servo_id, cmd_pwm)

		# pwm1_bytes = self.pwm2word(servo_list[0])
		# pwm2_bytes = self.pwm2word(servo_list[1])
		# pwm3_bytes = self.pwm2word(servo_list[2])
		# all_bytes = pwm1_bytes + pwm2_bytes + pwm3_bytes
		# self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, PWM1_H, all_bytes)

	def write_pwm_bytes(self, servo_id, pwm):

		pwm_bytes = self.pwm2word(pwm)

		if servo_id == 0:
			self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, PWM1_H, pwm_bytes)
		elif servo_id == 1:
			self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, PWM2_H, pwm_bytes)
		if servo_id == 2:
			self.bus.write_i2c_block_data(JMOAB_I2C_ADDR2, PWM3_H, pwm_bytes)

	#############################
	### Math helper functions ###
	#############################

	def sbus2word(self, sbus_val):

		high_byte = sbus_val >> 8
		low_byte = (sbus_val & 0x00FF)

		return [high_byte, low_byte]

	def pwm2word(self, pwm_val):
		high_byte = pwm_val >> 8
		low_byte = (pwm_val & 0x00FF)

		return [high_byte, low_byte]
	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def convert2voltage(self, raw_list):
		raw_list = np.asarray(raw_list)
		voltage_array =  self.map(raw_list, 0.0, 255.0, 0.0, 40.96)
		return voltage_array

	def over_limit_check(self, pwm):

		if pwm < self.pwm_min:
			pwm  = self.pwm_min
		elif pwm > self.pwm_max:
			pwm = self.pwm_max

		return pwm


	#############################
	### Cart mixing functions ###
	#############################
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

	############
	### Loop ###
	############
	def loop(self):

		rate = rospy.Rate(20) # 10hz

		while not rospy.is_shutdown():

			## Read ADC ##
			voltage_array = self.read_adc()
			adc_msg = Float32MultiArray()
			adc_msg.data = voltage_array

			## Read SBUS ch ##
			sbus_ch_array = self.read_sbus_ch()
			sbus_ch_msg = Int16MultiArray()
			sbus_ch_msg.data = sbus_ch_array

			## Read cart_mode ##
			cart_mode_num = self.read_atcart_mode()
			cart_mode_msg = UInt8()
			cart_mode_msg.data = cart_mode_num

			## Publish data ##
			self.adc_pub.publish(adc_msg)
			self.sbus_ch_pub.publish(sbus_ch_msg)
			self.cart_mode_pub.publish(cart_mode_msg)


			############################
			### cart control command ###
			############################
			if cart_mode_num == 2:
				cmd_vel_period = time.time() - self.cmd_vel_stamp
				wheels_cmd_period = time.time() - self.wheels_cmd_stamp

				## 1st priority is cmd_vel 
				if (cmd_vel_period < self.cmd_vel_timeout) and self.cmd_vel_cb_flag:

					if abs(self.vx) > 0.0 and abs(self.wz) > 0.0:
						y_percent = self.map(self.vx, -self.vx_max, self.vx_max, -100.0, 100.0)
						x_percent = self.map(self.wz, -self.wz_max, self.wz_max, y_percent, -y_percent)

						# if abs(x_percent) > abs(y_percent):
						# 	if x_percent > 0.0:
						# 		x_percent = y_percent
						# 	elif x_percent < 0.0:
						# 		x_percent = -y_percent
						
					else:
						x_percent = self.map(self.wz, -self.wz_max, self.wz_max, 100.0, -100.0)
						y_percent = self.map(self.vx, -self.vx_max, self.vx_max, -100.0, 100.0)

					left_200_per, right_200_per = self.xy_mixing(x_percent, y_percent)
					left_sbus, right_sbus = self.wheels_percent_to_wheels_sbus(left_200_per, right_200_per)
					self.write_left_right(int(left_sbus), int(right_sbus))

				## 2nd priority is wheels_cmd 
				elif (wheels_cmd_period < self.wheels_cmd_timeout) and self.wheels_cmd_cb_flag:
					x_percent = 0.0
					y_percent = 0.0

					left_200_per = self.left_percent*2.0
					right_200_per = self.right_percent*2.0

					left_sbus, right_sbus = self.wheels_percent_to_wheels_sbus(left_200_per, right_200_per)
					self.write_left_right(int(left_sbus), int(right_sbus))

				else:
					self.write_left_right(1024, 1024)
					self.cmd_vel_cb_flag = False
					self.wheels_cmd_cb_flag = False
					self.vx = 0.0
					self.wz = 0.0
					self.left_percent = 0.0
					self.right_percent = 0.0
					left_sbus = 1024
					right_sbus = 1024
					x_percent = 0.0
					y_percent = 0.0

				print("vx: {:.2f} wz: {:.2f} left: {:.2f} right: {:.2f} x: {:.2f} y: {:.2f} l_sbus: {:d} r_sbus: {:d}".format(\
					self.vx, self.wz, self.left_percent, self.right_percent, x_percent, y_percent, int(left_sbus), int(right_sbus)))

			elif cart_mode_num == 0:
				self.write_left_right(1024, 1024)
				self.cmd_vel_cb_flag = False
				self.wheels_cmd_cb_flag = False

			#########################
			### cart mode command ###
			#########################
			if self.cart_mode_cb_flag:
				self.write_atcart_mode(self.cart_mode_cmd)
				self.cart_mode_cb_flag = False

			#####################
			### relay command ###
			#####################
			if self.relay_cb_flag:
				self.write_relay(self.relay_list)
				self.relay_cb_flag = False

			#####################
			### servo command ###
			#####################
			if self.servo_cb_flag:
				self.write_servo(self.servo_list)
				self.servo_cb_flag = False


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
#! /usr/bin/env python3
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int16MultiArray, UInt8, Int8MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist
import time
import argparse
import numpy as np
from ddsm115 import *
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

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

class JMOAB_ATCart_Tiny:

	def __init__(self, NS):

		rospy.init_node('jmoab_ros_atcart_tiny_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ATCart-Tiny node")

		self.bus = SMBus(1)

		### DDSM115 ###
		self.drive = MotorControl()
		self.drive.set_drive_mode(_id=1, _mode=2)
		self.drive.set_drive_mode(_id=2, _mode=2)
		self.max_rpm = 300

		self.rev_str = False #False
		self.rev_thr = False

		self.L = 0.275
		self.R_wheel = 0.050

		self.period = 0.05
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0

		#### SBUS Steering Throttle ####
		self.sbus_steering_mid = 1024
		self.sbus_throttle_mid = 1024

		self.cmd_steering = self.sbus_steering_mid
		self.cmd_throttle = self.sbus_throttle_mid

		self.sbus_max = 1680.0
		self.sbus_min = 368.0
		self.sbus_mid = 1024.0
		self.sbus_max_DB = self.sbus_mid + 10.0
		self.sbus_min_DB = self.sbus_mid - 10.0

		self.sbus_min_backward = 975 #968	# a value before start rotating backward
		self.sbus_min_forward = 1085 #1093	# a value before start rotating forward
		self.prev_y = 0.0

		### cmd_vel
		self.vx_max = 2.0
		self.wz_max = 2.0
		self.vx = 0.0
		self.wz = 0.0
		self.cmd_vel_cb_flag = False
		self.cmd_vel_timeout = 1.0
		self.cmd_vel_stamp = time.time()

		### cart_mode_cmd
		self.cart_mode_cmd = 1
		self.cart_mode_cb_flag = False

		### relay cmd
		self.relay_list = []
		self.relay_cb_flag = False

		### servo cmd
		self.pwm_min = 820
		self.pwm_max = 2220
		self.pwm_mid = 1520
		self.servo_list = []
		self.servo_cb_flag = False

		#### Pub/Sub ####
		sbus_rc_topic = self.namespace_attaching(NS, "/jmoab/sbus_rc_ch")
		cart_mode_topic = self.namespace_attaching(NS, "/jmoab/cart_mode")
		cart_mode_cmd_topic = self.namespace_attaching(NS, "/jmoab/cart_mode_cmd")
		relay_topic = self.namespace_attaching(NS, "/jmoab/relays")
		adc_topic = self.namespace_attaching(NS, '/jmoab/adc')
		servo_topic = self.namespace_attaching(NS, '/jmoab/servos')

		cmd_vel_topic = self.namespace_attaching(NS, "/cmd_vel")

		self.odom_msg = Odometry()
		self.odom_pub = rospy.Publisher("/atcart_tiny/odom", Odometry, queue_size=10)

		self.sbus_ch_pub = rospy.Publisher(sbus_rc_topic, Int16MultiArray, queue_size=1)
		self.cart_mode_pub = rospy.Publisher(cart_mode_topic, UInt8, queue_size=1)
		self.adc_pub = rospy.Publisher(adc_topic, Float32MultiArray, queue_size=1)

		rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)
		rospy.Subscriber(cart_mode_cmd_topic, UInt8, self.cart_mode_callack)
		rospy.Subscriber(relay_topic, Int8MultiArray, self.relay_callback)
		rospy.Subscriber(servo_topic, Int16MultiArray, self.servo_callback)

		rospy.loginfo("Namespace is {}".format(NS))
		rospy.loginfo("Publishing to  {:} topic as std_msgs/Int16MultiArray".format(sbus_rc_topic))
		rospy.loginfo("Publishing to  {:} topic as std_msgs/UInt8".format(cart_mode_topic))
		rospy.loginfo("Publishing to  {:} topic as std_msgs/Float32MultiArray".format(adc_topic))

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

		self.vx = msg.linear.x
		self.wz = msg.angular.z

		self.cmd_vel_cb_flag = True
		self.cmd_vel_stamp = time.time()

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

	def linear_to_rpm(self, v):
		rpm = (60.0/(2.0*np.pi))*(v/self.R_wheel)
		return rpm

	def rpm_to_radPerSec(self, rpm):
		return rpm*2*np.pi/60.0

	def rpm_to_linear(self, rpm):

		W_Wheel = self.rpm_to_radPerSec(rpm)
		V = W_Wheel*self.R_wheel

		return V

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

	def wheels_percent_to_wheels_rpm(self, left_per, right_per):

		if left_per > 0.0:
			left_rpm = self.map(left_per, 0.0, 200.0, 0.0, self.max_rpm)
		elif left_per < 0.0:
			left_rpm = self.map(left_per, -200.0, 0.0, -self.max_rpm, 0.0)
		else:
			left_rpm = 0.0

		if right_per > 0.0:
			right_rpm = self.map(right_per, 0.0, 200.0, 0.0, -self.max_rpm)
		elif right_per < 0.0:
			right_rpm = self.map(right_per, -200.0, 0.0, self.max_rpm, 0.0)
		else:
			right_rpm = 0.0

		return left_rpm, right_rpm

	def write_rpm(self, left_rpm, right_rpm):
		self.drive.send_rpm(_id=1, rpm=int(left_rpm))
		self.drive.send_rpm(_id=2, rpm=int(right_rpm))

	def sbus_db_check(self, sbus):

		if self.sbus_min_DB < sbus < self.sbus_max_DB:
			return 1024
		else:
			return sbus

	def sbusCmds_to_RPMs(self, sbus_steering, sbus_throttle):
		# x_percent = self.sbus2percent(sbus_steering)
		# y_percent = self.sbus2percent(sbus_throttle)
		if self.rev_thr:
			y = self.map(sbus_throttle, self.sbus_min, self.sbus_max, 100.0, -100.0)
		else:
			y = self.map(sbus_throttle, self.sbus_min, self.sbus_max, -100.0, 100.0)

		if self.rev_str:
			x = self.map(sbus_steering, self.sbus_min, self.sbus_max, 100.0, -100.0)
		else:
			x = self.map(sbus_steering, self.sbus_min, self.sbus_max, -100.0, 100.0)

		left_percent, right_percent = self.xy_mixing(x, y)
		left_rpm = self.map(left_percent, -200.0, 200.0, -self.max_rpm, self.max_rpm)
		right_rpm = self.map(right_percent, -200.0, 200.0, self.max_rpm, -self.max_rpm)

		return left_rpm, right_rpm



	############
	### Loop ###
	############
	def loop(self):

		rate = rospy.Rate(20) # 10hz
		last_log_stamp = time.time()
		vl = 0.0
		vr = 0.0
		left_rpm = 0.0
		right_rpm = 0.0
		while not rospy.is_shutdown():

			start_time = time.time()

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

				## 1st priority is cmd_vel 
				if (cmd_vel_period < self.cmd_vel_timeout) and self.cmd_vel_cb_flag:

					if (self.vx != 0.0) and (self.wz == 0.0):
						vl = self.vx
						vr = self.vx

					elif (self.vx == 0.0) and (self.wz != 0.0):

						vl = -self.wz * self.L/2.0
						vr = self.wz * self.L/2.0

					elif (self.vx != 0.0) and (self.wz != 0.0):
						R_icc = abs(self.vx)/abs(self.wz)
						sign_vx = self.vx/abs(self.vx)
						if self.wz > 0.0:
							# print("curve left")
							if self.vx > 0.0:
								vl = (sign_vx)*(self.wz*(R_icc - self.L/2.0)) #/2.0
								vr = (sign_vx)*(self.wz*(R_icc + self.L/2.0)) #/2.0
							else:
								vl = (sign_vx)*(self.wz*(R_icc + self.L/2.0)) #/2.0
								vr = (sign_vx)*(self.wz*(R_icc - self.L/2.0)) #/2.0
						elif self.wz < 0.0:
							# print("curve right")
							if self.vx > 0.0:
								vl = (sign_vx)*(abs(self.wz)*(R_icc + self.L/2.0)) #/2.0
								vr = (sign_vx)*(abs(self.wz)*(R_icc - self.L/2.0)) #/2.0
							else:
								vl = (sign_vx)*(abs(self.wz)*(R_icc - self.L/2.0)) #/2.0
								vr = (sign_vx)*(abs(self.wz)*(R_icc + self.L/2.0)) #/2.0
					else:
						vl = 0.0
						vr = 0.0

					left_rpm = int(self.linear_to_rpm(vl))
					right_rpm = int(self.linear_to_rpm(-vr))

					self.write_rpm(left_rpm, right_rpm)

				else:
					self.write_rpm(0, 0)
					self.cmd_vel_cb_flag = False
					self.vx = 0.0
					self.wz = 0.0

					left_rpm = 0.0
					right_rpm = 0.0
	
			elif cart_mode_num == 1:
				left_rpm, right_rpm = self.sbusCmds_to_RPMs(self.sbus_db_check(sbus_ch_array[0]), self.sbus_db_check(sbus_ch_array[1]))
				self.write_rpm(left_rpm, right_rpm)
				self.cmd_vel_cb_flag = False

			else:
				self.write_rpm(0, 0)
				self.cmd_vel_cb_flag = False

			############################
			### Odometry computation ###
			############################
			rpm_L, _ = self.drive.get_motor_feedback(_id=1)
			rpm_R, _ = self.drive.get_motor_feedback(_id=2)

			VL = self.rpm_to_linear(rpm_L)
			VR = self.rpm_to_linear(-rpm_R)

			VL = round(VL, 3)
			VR = round(VR, 3)
			if (VL > 0.0) and (VR < 0.0) and (abs(VL) == abs(VR)):
				## rotatiing CW
				V = -VL
				Wz = 2.0*V/self.L
				self.theta = self.theta + Wz*self.period

				path = "skid_right"

			elif (VR > 0.0) and (VL < 0.0) and (abs(VL) == abs(VR)):
				## rotatiing CCW
				V = VR
				Wz = 2.0*V/self.L
				self.theta = self.theta + Wz*self.period

				path = "skid_left"

			elif abs(VL) > abs(VR):
				## curving CW
				V = (VL + VR)/2.0
				Wz = (VL-VR)/self.L
				R_ICC = (self.L/2.0)*((VL+VR)/(VL-VR))

				self.x = self.x - R_ICC*np.sin(self.theta) + R_ICC*np.sin(self.theta + Wz*self.period)
				self.y = self.y + R_ICC*np.cos(self.theta) - R_ICC*np.cos(self.theta + Wz*self.period)
				self.theta = self.theta - Wz*self.period

				path = "curve_right"

			elif abs(VL) < abs(VR):
				## curving CCW
				V = (VL + VR)/2.0
				Wz = (VR-VL)/self.L
				R_ICC = (self.L/2.0)*((VR+VL)/(VR-VL))

				self.x = self.x - R_ICC*np.sin(self.theta) + R_ICC*np.sin(self.theta + Wz*self.period)
				self.y = self.y + R_ICC*np.cos(self.theta) - R_ICC*np.cos(self.theta + Wz*self.period)
				self.theta = self.theta + Wz*self.period

				path = "curve_left"

			elif VL == VR:
				V = (VL + VR)/2.0
				Wz = 0.0
				self.x = self.x + V*np.cos(self.theta)*self.period
				self.y = self.y + V*np.sin(self.theta)*self.period
				self.theta = self.theta
				path = "straight"

			else:
				V = 0.0
				Wz = 0.0
				R_ICC = 0.0

			q = quaternion_from_euler(0,0, self.theta)

			self.odom_msg.header.stamp = rospy.Time.now()
			self.odom_msg.header.frame_id = "odom"
			self.odom_msg.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
			self.odom_msg.pose.pose.position.x = self.x
			self.odom_msg.pose.pose.position.y = self.y
			self.odom_msg.pose.pose.position.z = 0.0
			self.odom_msg.pose.pose.orientation.x = q[0]
			self.odom_msg.pose.pose.orientation.y = q[1]
			self.odom_msg.pose.pose.orientation.z = q[2]
			self.odom_msg.pose.pose.orientation.w = q[3]
			self.odom_msg.pose.covariance[0] = 0.0001
			self.odom_msg.pose.covariance[7] = 0.0001
			self.odom_msg.pose.covariance[14] = 0.000001	#1e12
			self.odom_msg.pose.covariance[21] = 0.000001	#1e12
			self.odom_msg.pose.covariance[28] = 0.000001	#1e12
			self.odom_msg.pose.covariance[35] = 0.0001
			self.odom_msg.twist.twist.linear.x = V #vx_odom #V
			self.odom_msg.twist.twist.linear.y = 0.0 #vy_odom #0.0
			self.odom_msg.twist.twist.angular.z = Wz
			self.odom_pub.publish(self.odom_msg)


			#########################
			### Logging on screen ###
			#########################
			if (time.time() - last_log_stamp) > 1.0:
				if cart_mode_num == 1:
					print("str: {:d} thr: {:d} l_rpm: {:d} r_rpm: {:d} ".format(\
						sbus_ch_array[0], sbus_ch_array[1], int(left_rpm), int(right_rpm)))
				elif cart_mode_num == 2:
					print("vx: {:.2f} wz: {:.2f} vl: {:.2f} vr: {:.2f} rpmL: {:d} rpmR: {:d}".format(\
						self.vx, self.wz, vl, vr, int(left_rpm), int(right_rpm)))

				print("period: {:.4f} rpm_L: {:.2f} rpm_R: {:.2f} vl: {:.2f} vr: {:.2f}".format(\
					self.period, rpm_L, rpm_R, VL, VR))
		
				last_log_stamp = time.time()

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


			# self.period = time.time() - start_time


			rate.sleep()


if __name__ == '__main__':

	parser = argparse.ArgumentParser(description='ATCart Tiny diff-drive for jmoab-ros')
	parser.add_argument('--ns',
						help="a namespace in front of topics")

	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	ns = args.ns

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")


	jmoab = JMOAB_ATCart_Tiny(ns)
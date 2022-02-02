#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray, Int8, Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
from ZLAC8015D import *
import time

class JMOAB_ZLAC8015D:

	def __init__(self):

		rospy.init_node('jmoab_ros_ZLAC8015D_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ZLAC8015D node")

		self.bus = SMBus(1)

		self.sbus_ch_pub = rospy.Publisher("/sbus_rc_ch", Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()

		self.atcart_mode_pub = rospy.Publisher("/atcart_mode", Int8, queue_size=10)
		self.atcart_mode = Int8()

		self.wheels_rpm_pub = rospy.Publisher("/wheels_rpm", Float32MultiArray, queue_size=10)
		# time.sleep(1)
		self.wheels_rpm_msg = Float32MultiArray()

		self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
		self.odom_msg = Odometry()


		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.sbus_cmd_callback)
		rospy.Subscriber("/atcart_mode_cmd", Int8, self.cart_mode_callack)
		rospy.Subscriber("/zlac8015d/mode_cmd", Int8, self.zlac8015d_mode_callack)
		# rospy.Subscriber("/zlac8015d/vel/rpm_cmd", Float32MultiArray, self.zlac8015d_rpm_cmd_callback)
		rospy.Subscriber("/zlac8015d/pos/deg_cmd", Float32MultiArray, self.zlac8015d_deg_cmd_callback)
		rospy.Subscriber("/zlac8015d/pos/dist_cmd", Float32MultiArray, self.zlac8015d_dist_cmd_callback)


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

		self.rev_str = False #False
		self.rev_thr = True

		#### JMOAB I2C REG ####
		self.ATCART_MODE_REG = 0x52
		self.SBUS_FS_REG = 0x57

		self.mode = "AUTO"
		self.mode_num = 2
		self.prev_mode_num = 3

		### ZLAC8015D Init ###
		self.zlc = ZLAC8015D()
		self.zlac8015d_speed_mode_init()

		self.max_rpm = 150
		self.ultimate_rpm = 200 
		self.deadband_rpm = 3

		self.zlac8015d_mode = 3

		self.read_mode_period = 1.0
		self.left_pos_deg_cmd  = 0.0
		self.right_pos_deg_cmd = 0.0
		self.got_pos_cmd = False
		self.l_meter = 0.0
		self.r_meter = 0.0

		self.left_pos_dist_cmd = 0.0
		self.right_pos_dist_cmd = 0.0
		self.got_pos_dist_cmd = False

		self.travel_in_one_rev = self.zlc.travel_in_one_rev

		self.l_meter_init, self.r_meter_init = self.zlc.get_wheels_travelled()
		self.prev_l_meter = 0.0
		self.prev_r_meter = 0.0

		self.L = 0.440

		self.br = tf2_ros.TransformBroadcaster()
		self.t = TransformStamped()

		rospy.loginfo("Publishing SBUS RC channel on /sbus_rc_ch topic")
		rospy.loginfo("Subscribing on /sbus_cmd topic for steering and throttle values")
		rospy.loginfo("Publishing ATCart mode on /atcart_mode topic")
		rospy.loginfo("Subscribing on /atcart_mode_cmd topic for mode changing")

		rospy.loginfo("Publishing wheels rpm on /wheels_rpm topic")

		self.loop()

		rospy.spin()

	def zlac8015d_speed_mode_init(self):
		self.zlc.disable_motor()
		self.zlc.set_accel_time(200,200)
		self.zlc.set_decel_time(200,200)
		self.zlc.set_mode(3)
		self.zlc.enable_motor()

	def zlac8015d_position_mode_init(self):
		self.zlc.disable_motor()
		self.zlc.set_accel_time(500,500)
		self.zlc.set_decel_time(500,500)
		self.zlc.set_mode(1)
		self.zlc.set_position_async_control()
		self.zlc.set_maxRPM_pos(50,50)
		self.zlc.enable_motor()

	def zlac8015d_mode_callack(self, msg):
		self.zlac8015d_mode = msg.data
		if (self.zlac8015d_mode == 1):
			self.zlac8015d_position_mode_init()
		elif (self.zlac8015d_mode == 3):
			self.zlac8015d_speed_mode_init()

		self.mode = "AUTO"
		self.mode_num = 2
		self.write_atcart_mode(self.mode_num)

	def zlac8015d_deg_cmd_callback(self, msg):

		self.left_pos_deg_cmd = msg.data[0]
		self.right_pos_deg_cmd = msg.data[1]
		self.got_pos_cmd = True

	def zlac8015d_dist_cmd_callback(self, msg):

		self.left_pos_dist_cmd = msg.data[0]
		self.right_pos_dist_cmd = msg.data[1]
		self.got_pos_dist_cmd = True

	def zlac8015d_set_rpm_with_limit(self, left_rpm, right_rpm):
		if (-self.ultimate_rpm < left_rpm < self.ultimate_rpm) and (-self.ultimate_rpm < right_rpm < self.ultimate_rpm): 
			# print(left_rpm, right_rpm)
			if (-self.deadband_rpm < left_rpm < self.deadband_rpm): 
				left_rpm = 0

			if (-self.deadband_rpm < right_rpm < self.deadband_rpm): 
				right_rpm = 0

			self.zlc.set_rpm(int(left_rpm), int(right_rpm))


	def write_atcart_mode(self, mode_num):
		## need to write multiple times to take effect
		## publisher side only sends one time is ok
		for i in range(10):
			self.bus.write_byte_data(0x71, self.ATCART_MODE_REG, mode_num)

	def read_atcart_mode(self):
		self.mode_num = self.bus.read_byte_data(0x71, self.ATCART_MODE_REG)
		if self.prev_mode_num != self.mode_num:
			if self.mode_num == 0:
				self.mode = "HOLD"
			elif self.mode_num == 1:
				self.mode = "MANUAL"
				self.zlac8015d_speed_mode_init()	# manual mode is only supported speed control
			elif self.mode_num == 2:
				self.mode = "AUTO"
			else:
				self.mode = "UNKNOWN"

			self.zlac8015d_mode = self.zlc.get_mode()

		self.prev_mode_num = self.mode_num

		return self.mode_num 

	def cart_mode_callack(self, msg):
		self.write_atcart_mode(msg.data)


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


		if (self.prev_Y < 0.0):
			swap = left
			left = right
			right = swap

		self.prev_Y = y

		left_rpm = self.map(left, -200.0, 200.0, self.max_rpm, -self.max_rpm)
		right_rpm = self.map(right, -200.0, 200.0, -self.max_rpm, self.max_rpm)

		return int(left_rpm), int(right_rpm)


	def loop(self):

		rate = rospy.Rate(20) # 10hz
		prev_ch5 = 1024
		left_rpm = 0
		right_rpm = 0
		period = 0.05
		stamp_read_mode = time.time()
		x = 0.0
		y = 0.0
		theta= 0.0
		V = 0.0
		Wz = 0.0

		while not rospy.is_shutdown():

			start_time = time.time()

			sbus_ch_array = self.get_sbus_channel()


			self.sbus_ch.data = sbus_ch_array
			self.sbus_ch_pub.publish(self.sbus_ch)


			if self.mode_num == 2:

				## Speed control mode
				if self.zlac8015d_mode == 3:
					if ((time.time() - self.callback_timestamp) > self.callback_timeout):
						left_rpm = 0
						right_rpm = 0
						self.cmd_steering = 1024
						self.cmd_throttle = 1024
					else:
						left_rpm, right_rpm = self.channel_mixing(self.cmd_steering, self.cmd_throttle)
						
					self.mode_num = 2
					self.mode = "AUTO"
					self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

					fb_L_rpm, fb_R_rpm = self.zlc.get_rpm()
					self.wheels_rpm_msg.data = [fb_L_rpm, fb_R_rpm]
					self.wheels_rpm_pub.publish(self.wheels_rpm_msg)


				# Position control mode 
				elif self.zlac8015d_mode == 1:
					if self.got_pos_cmd:
						self.zlc.set_relative_angle(self.left_pos_deg_cmd ,self.right_pos_deg_cmd)
						self.zlc.move_left_wheel()
						self.zlc.move_right_wheel()
						self.got_pos_cmd = False

					elif self.got_pos_dist_cmd:
						left_cmd_deg = (self.left_pos_dist_cmd*360.0)/self.travel_in_one_rev
						right_cmd_deg = (-self.right_pos_dist_cmd*360.0)/self.travel_in_one_rev
						self.zlc.set_relative_angle(left_cmd_deg, right_cmd_deg)
						self.zlc.move_left_wheel()
						self.zlc.move_right_wheel()
						self.got_pos_dist_cmd = False


			elif self.mode_num == 1:
				left_rpm, right_rpm = self.channel_mixing(sbus_ch_array[0], sbus_ch_array[1])
				self.mode = "MANUAL"
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
				self.mode_num = 1
				self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

				fb_L_rpm, fb_R_rpm = self.zlc.get_rpm()
				self.wheels_rpm_msg.data = [fb_L_rpm, fb_R_rpm]
				self.wheels_rpm_pub.publish(self.wheels_rpm_msg)


			elif self.mode_num == 0:
				left_rpm = 0
				right_rpm = 0
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
				self.mode = "HOLD"
				self.mode_num = 0
				self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

			else:
				self.mode = "UNKNOWN"
				left_rpm = 0
				right_rpm = 0
				self.cmd_steering = 1024
				self.cmd_throttle = 1024
				self.mode_num = 3
				self.zlac8015d_set_rpm_with_limit(left_rpm, right_rpm)

			############################
			### Odometry computation ###
			############################
			self.l_meter, self.r_meter = self.zlc.get_wheels_travelled()
			self.l_meter = self.l_meter - self.l_meter_init 
			self.r_meter = (-1*self.r_meter) - (-1*self.r_meter_init)
			if self.zlac8015d_mode == 1:
				vl = (self.l_meter-self.prev_l_meter)/period
				vr = (self.r_meter-self.prev_r_meter)/period
			elif self.zlac8015d_mode == 3:
				vl,vr = self.zlc.get_linear_velocities()

			vl = round(vl, 3)
			vr = round(vr, 3)
			if (vl > 0.0) and (vr < 0.0) and (abs(vl) == abs(vr)):
				## rotatiing CW
				V = -vl
				Wz = 2.0*V/self.L
				theta = theta + Wz*period

				path = "skid_right"

			elif (vr > 0.0) and (vl < 0.0) and (abs(vl) == abs(vr)):
				## rotatiing CCW
				V = vr
				Wz = 2.0*V/self.L
				theta = theta + Wz*period

				path = "skid_left"

			elif abs(vl) > abs(vr):
				## curving CW
				V = (vl + vr)/2.0
				Wz = (vl-vr)/self.L
				R_ICC = (self.L/2.0)*((vl+vr)/(vl-vr))

				x = x - R_ICC*np.sin(theta) + R_ICC*np.sin(theta + Wz*period)
				y = y + R_ICC*np.cos(theta) - R_ICC*np.cos(theta + Wz*period)
				theta = theta - Wz*period

				path = "curve_right"

			elif abs(vl) < abs(vr):
				## curving CCW
				V = (vl + vr)/2.0
				Wz = (vr-vl)/self.L
				R_ICC = (self.L/2.0)*((vr+vl)/(vr-vl))

				x = x - R_ICC*np.sin(theta) + R_ICC*np.sin(theta + Wz*period)
				y = y + R_ICC*np.cos(theta) - R_ICC*np.cos(theta + Wz*period)
				theta = theta + Wz*period

				path = "curve_left"

			elif vl == vr:
				V = (vl + vr)/2.0
				Wz = 0.0
				x = x + V*np.cos(theta)*period
				y = y + V*np.sin(theta)*period
				theta = theta
				path = "straight"

			else:
				V = 0.0
				Wz = 0.0
				R_ICC = 0.0
				
			q = quaternion_from_euler(0,0, theta)
			# construct tf
			self.t.header.frame_id = "odom" 
			self.t.header.stamp = rospy.Time.now()
			self.t.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
			self.t.transform.translation.x = x
			self.t.transform.translation.y = y
			self.t.transform.translation.z = 0.0

			self.t.transform.rotation.x = q[0]
			self.t.transform.rotation.y = q[1]
			self.t.transform.rotation.z = q[2]
			self.t.transform.rotation.w = q[3]
			self.br.sendTransform(self.t)

			self.odom_msg.header.stamp = rospy.Time.now()
			self.odom_msg.header.frame_id = "odom"
			self.odom_msg.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
			self.odom_msg.pose.pose.position.x = x
			self.odom_msg.pose.pose.position.y = y
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
			self.odom_msg.twist.twist.linear.x = V
			self.odom_msg.twist.twist.linear.y = 0.0
			# self.odom_msg.twist.covariance[0] = 0.001
			# self.odom_msg.twist.covariance[7] = 0.001
			# self.odom_msg.twist.covariance[14] = 1e9
			# self.odom_msg.twist.covariance[21] = 1e9
			# self.odom_msg.twist.covariance[28] = 1e9
			# self.odom_msg.twist.covariance[35] = 0.001
			self.odom_msg.twist.twist.angular.z = Wz
			self.odom_pub.publish(self.odom_msg)


			#########################
			### Logging to screen ###
			#########################
			if self.mode != "AUTO":
				print("cart_mode: {:} | mode_num: {:d} | zlac_mode: {:d} | sbus_str: {:d} | sbus_thr: {:d} | left_rpm: {:d} | right_rpm: {:d} | l_meter: {:.2f} | r_meter: {:.2f} | VL: {:.3f} | VR: {:.3f}".format(\
					self.mode, self.mode_num, self.zlac8015d_mode, sbus_ch_array[0], sbus_ch_array[1], left_rpm, right_rpm, self.l_meter, self.r_meter, vl, vr))
			else:
				print("cart_mode: {:} | mode_num: {:d} | zlac_mode: {:d} | sbus_str: {:d} | sbus_thr: {:d} | left_rpm: {:d} | right_rpm: {:d} | l_meter: {:.2f} | r_meter: {:.2f} | VL: {:.3f} | VR: {:.3f}".format(\
					self.mode, self.mode_num, self.zlac8015d_mode, self.cmd_steering, self.cmd_throttle, left_rpm, right_rpm, self.l_meter, self.r_meter, vl, vr))


			self.atcart_mode.data =  self.read_atcart_mode() #self.mode_num
			self.atcart_mode_pub.publish(self.atcart_mode)

			prev_ch5 = sbus_ch_array[4]

			
			period = time.time() - start_time
			self.prev_l_meter = self.l_meter
			self.prev_r_meter = self.r_meter

			# print(period)

			rate.sleep()


if __name__ == '__main__':
	jmoab = JMOAB_ZLAC8015D()
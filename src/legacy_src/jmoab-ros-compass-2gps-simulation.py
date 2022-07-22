#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int8, Bool
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistWithCovarianceStamped
import time

class Imu2Compass:

	def __init__(self):

		rospy.init_node('jmoab_ros_imu_to_compass', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-IMU_COMPASS node")

		self.roll = 0.0
		self.pitch = 0.0
		self.heading = 0.0

		####################
		## heading offset ##
		####################
		self.hdg_offset = 0.0
		rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))
		self.lat = 0.0
		self.lon = 0.0
		self.calib_flag = False
		self.get_latlon_once = True
		self.ch7_from_high = False
		self.start_hdg = 0.0

		self.sbus_steering = 1024
		self.sbus_throttle = 1160

		self.fix_stat = 0
		self.sbus_steering_stick = 0.0
		self.sbus_throttle_stick = 0.0

		self.prev_lat = self.lat
		self.prev_lon = self.lon
		self.cart_mode = 1

		self.sbus_cmd_throttle = 1024
		self.sbus_cmd_steering = 1024

		self.pure_hdg = 0.0
		self.sign = 1.0
		self.brg = 0.0
		self.last_time_manual_cal = time.time()
		self.last_time_auto_cal = time.time()
		self.cal_offset = False
		self.do_estimation = False

		## Kalman filter of hdg_offset estimate
		self.hdg_off_est = 5.0 # first guess
		self.state_predict = self.hdg_off_est
		self.error_est = 10.0 
		self.error_mea = 8.0 # a variance of 

		self.true_hdg_from_2gps = 0.0
		self.lat2 = 0.0
		self.lon2 = 0.0
		self.fix_stat2 = 0


		rospy.Subscriber("/imu", Imu, self.imu_callback)
		
		self.compass_pub = rospy.Publisher('/jmoab_compass', Float32MultiArray, queue_size=10)
		self.compass_msg = Float32MultiArray()
		self.hdg_calib_flag_pub = rospy.Publisher("/hdg_calib_flag", Bool, queue_size=10)
		self.hdg_calib_flag_msg = Bool()
		# self.sbus_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=10)
		# self.sbus_cmd = Int32MultiArray()
		self.atcart_mode_cmd_pub = rospy.Publisher("/atcart_mode_cmd", Int8, queue_size=10)
		self.atcart_mode_cmd_msg = Int8()

		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/ublox2/fix", NavSatFix, self.gps2_callback)
		rospy.Subscriber("/atcart_mode", Int8, self.atcart_mode_callback)
		rospy.Subscriber("joy", Joy, self.joy_callback)

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.sbus_cmd_callback)

		self.loop()

		rospy.spin()

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg

	def ConvertTo180Range(self, deg):

		deg = self.ConvertTo360Range(deg)
		if deg > 180.0:
			deg = -(180.0 - (deg%180.0))

		return deg

	def find_smallest_diff_ang(self, goal, cur):

		## goal is in 180ranges, we need to convert to 360ranges first

		diff_ang1 = abs(self.ConvertTo360Range(goal) - cur)

		if diff_ang1 > 180.0:

			diff_ang = 180.0 - (diff_ang1%180.0)
		else:
			diff_ang = diff_ang1

		## check closet direction
		compare1 = self.ConvertTo360Range(self.ConvertTo360Range(goal) - self.ConvertTo360Range(cur + diff_ang))
		compare2 = self.ConvertTo180Range(goal - self.ConvertTo180Range(cur + diff_ang))
		# print(compare1, compare2)
		if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
			sign = 1.0 # clockwise count from current hdg to target
		else:
			sign = -1.0 # counter-clockwise count from current hdg to target


		return diff_ang, sign

	def joy_callback(self, msg):

		## Y button on Logicool
		if msg.buttons[3] == 1:
			self.calib_flag = True
		elif msg.buttons[3] == 0:
			self.calib_flag = False

		self.sbus_steering_stick = msg.axes[3]
		self.sbus_throttle_stick = msg.axes[1]

	def gps_callback(self, msg):

		self.prev_lat = self.lat
		self.prev_lon = self.lon

		self.lat = msg.latitude
		self.lon = msg.longitude

		self.fix_stat = msg.status.status

		###########################################################################
		## we put the calculation/estimation inside the gps_callback             ##
		## because the sampling time of gps is slowest,                          ##
		## so we could get the real difference between current and previous data ##
		###########################################################################
		if self.do_estimation:
			# self.brg = self.get_bearing(self.prev_lat, self.prev_lon, self.lat, self.lon)
			# self.brg = self.ConvertTo360Range(self.brg)

			if self.cart_mode == 1:

				self.period_manual_cal = time.time() - self.last_time_manual_cal

				if self.period_manual_cal > 1.0:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.true_hdg_from_2gps, self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)
			elif self.cart_mode == 2:

				self.period_auto_cal = time.time() - self.last_time_auto_cal

				if self.period_auto_cal > 0.3:
					self.hdg_offset, self.sign = self.find_smallest_diff_ang(self.true_hdg_from_2gps, self.pure_hdg)
					self.cal_offset = True
					self.hdg_off_est = self.kalman_filter(self.hdg_offset, self.state_predict, self.error_est)

	def gps2_callback(self, msg):

		self.lat2 = msg.latitude
		self.lon2 = msg.longitude
		self.fix_stat2 = msg.status.status

	def atcart_mode_callback(self, msg):
		self.cart_mode = msg.data

	def sbus_cmd_callback(self, msg):

		self.sbus_cmd_steering = msg.data[0]
		self.sbus_cmd_throttle = msg.data[1]


	def get_bearing(self, lat1, lon1, lat2, lon2):

		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		y = np.sin(dLon)*np.cos(lat_end)
		x = np.cos(lat_start)*np.sin(lat_end) - np.sin(lat_start)*np.cos(lat_end)*np.cos(dLon)
		bearing = np.degrees(np.arctan2(y,x))

		return bearing


	def imu_callback(self,data):

		qw = data.orientation.w
		qx = data.orientation.x
		qy = data.orientation.y
		qz = data.orientation.z

		r11 = qw**2 + qx**2 - qy**2 - qz**2 #1 - 2*qy**2 - 2*qz**2
		r12 = 2*qx*qy - 2*qz*qw
		r13 = 2*qx*qz + 2*qy*qw
		r21 = 2*qx*qy + 2*qz*qw
		r22 = qw**2 - qx**2 + qy**2 - qz**2	#1 - 2*qx**2 - 2*qz**2
		r23 = 2*qy*qz - 2*qx*qw
		r31 = 2*qx*qz - 2*qy*qw
		r32 = 2*qy*qz + 2*qx*qw
		r33 = qw**2 - qx**2 - qy**2 + qz**2	#1 - 2*qx**2 - 2*qy**2
		rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

		## default orientation (heatsink is pointing down)
		rot_matrix = np.array([
								[ 1, 0, 0],
								[ 0, 1, 0],
								[ 0, 0, 1]])

		## board is pitch 180 (heatsink is pointing up)
		# rot_matrix = np.array([
		# 						[-1, 0, 0],
		# 						[ 0, 1, 0],
		# 						[ 0, 0,-1]])

		rot = np.dot(rot,rot_matrix)

		self.heading = self.ConvertTo360Range(np.degrees(-np.arctan2(rot[1,0],rot[0,0])))
		self.pitch = np.degrees(np.arcsin(-rot[2,0]))
		self.roll = np.degrees(np.arctan2(rot[2,1],rot[2,2]))

	def kalman_filter(self, measure, prev_state_est, prev_error_est):

		## reference
		## https://www.kalmanfilter.net/kalman1d.html

		######################
		## state estimation ##
		######################
		KG = prev_error_est/(prev_error_est + self.error_mea)
		cur_state_est = prev_state_est + KG*(measure - prev_state_est)
		cur_error_est = (1 - KG)*(prev_error_est)

		################
		## prediction ##
		################
		self.error_est = cur_error_est + 0.1 # 0.01 is process noise, could help when hdg_offset is fluctuating during time
		# hdg_offset is not dynamic behaviour, so predicted state is constant as estimated state
		self.state_predict = cur_state_est  

		return cur_state_est



	def loop(self):

		rate = rospy.Rate(100)

		hdg = 0.0
		while not rospy.is_shutdown():

			self.pure_hdg = self.heading
			# hdg = self.ConvertTo360Range(self.heading - self.hdg_offset)
			# hdg = self.ConvertTo360Range(self.heading + (sign)*self.hdg_offset)
			hdg = self.ConvertTo360Range(self.heading + (self.sign)*self.hdg_off_est)

			#############################################################
			## hdg_offset calculation by two points gps bearing angle, ##
			## and estimation by Kalman Filter                         ##
			#############################################################
			print(self.lat, self.lon, self.lat2, self.lon2)
			self.true_hdg_from_2gps = self.ConvertTo360Range(self.get_bearing(self.lat, self.lon, self.lat2, self.lon2))
			print(self.true_hdg_from_2gps)
			## must be rtk-fixed, to get precise position
			if self.fix_stat == 2:
				## in manual case
				if self.cart_mode == 1:
					###############################################################
					## we do estimation only when throttle is up and no steering ##
					###############################################################
					if (-0.1 < self.sbus_throttle_stick) and (-0.07 < self.sbus_steering_stick < 0.07):
						
						self.do_estimation = True

					else:
						self.last_time_manual_cal = time.time()
						self.last_time_auto_cal = time.time()
						self.cal_offset = False
						self.do_estimation = False

				## in auto case
				elif self.cart_mode == 2:
					#######################################################
					## we do estimation only when sbus throttle is high, ##
					## and sbus steering is no curvy                     ##
					#######################################################
					if (1000 < self.sbus_cmd_throttle) and (1000 < self.sbus_cmd_steering < 1048):

						self.do_estimation = True

					else:
						self.last_time_manual_cal = time.time()
						self.last_time_auto_cal = time.time()
						self.cal_offset = False
						self.do_estimation = False

				else:
					self.last_time_manual_cal = time.time()
					self.last_time_auto_cal = time.time()

			print("pure_hdg: {:.2f} | true_hdg: {:.2f} | hdg_off: {:.2f} | hdg_off_est: {:.2f} | hdg: {:.2f} | cal_off: {:}".format(\
				self.pure_hdg, self.true_hdg_from_2gps, self.hdg_offset, self.hdg_off_est, hdg, self.cal_offset))

			self.compass_msg.data = [self.roll, self.pitch, hdg]
			self.compass_pub.publish(self.compass_msg)



			rate.sleep()


if __name__ == '__main__':
	a = Imu2Compass()
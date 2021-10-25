#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int8, Bool
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Joy
import time

class Imu2Compass:

	def __init__(self):

		rospy.init_node('jmoab_ros_imu_to_compass', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-IMU_COMPASS node")

		rospy.Subscriber("/imu", Imu, self.imu_callback)
		
		self.compass_pub = rospy.Publisher('/jmoab_compass', Float32MultiArray, queue_size=10)
		self.compass_msg = Float32MultiArray()
		self.hdg_calib_flag_pub = rospy.Publisher("/hdg_calib_flag", Bool, queue_size=10)
		self.hdg_calib_flag_msg = Bool()
		self.sbus_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()
		self.atcart_mode_cmd_pub = rospy.Publisher("/atcart_mode_cmd", Int8, queue_size=10)
		self.atcart_mode_cmd_msg = Int8()

		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/atcart_mode", Int8, self.atcart_mode_callback)
		rospy.Subscriber("joy", Joy, self.joy_callback)

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

		self.loop()

		rospy.spin()

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg

	def joy_callback(self, msg):

		## Y button on Logicool
		if msg.buttons[3] == 1:
			self.calib_flag = True
		elif msg.buttons[3] == 0:
			self.calib_flag = False

	def gps_callback(self, msg):

		self.lat = msg.latitude
		self.lon = msg.longitude

	def atcart_mode_callback(self, msg):
		self.cart_mode = msg.data

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


	def loop(self):

		rate = rospy.Rate(100)

		hdg = 0.0
		pure_hdg = 0.0

		while not rospy.is_shutdown():

			pure_hdg = self.heading
			hdg = self.ConvertTo360Range(self.heading - self.hdg_offset)


			#######################################
			### Live heading offset calibration ###
			#######################################
			if self.calib_flag:
				if self.get_latlon_once:
					lat_start = self.lat
					lon_start = self.lon
					self.get_latlon_once = False
					self.hdg_offset = 0.0
					# hdg = self.ConvertTo360Range(hdg-self.hdg_offset)
					self.start_hdg = pure_hdg
					rospy.loginfo("Start heading offset calibrating with start_hdg {:.2f} hdg_offset {:.2f}".format(self.start_hdg, self.hdg_offset))

					######################################################################
					## Set hdg_calib_flag to True to let other autopilot node knows     ##
					## Set atcart_mode to auto, we're gonna use auto mode straight line ##
					######################################################################
					self.hdg_calib_flag_msg.data = True
					self.hdg_calib_flag_pub.publish(self.hdg_calib_flag_msg)
					self.atcart_mode_cmd_msg.data = 2
					self.atcart_mode_cmd_pub.publish(self.atcart_mode_cmd_msg)
					time.sleep(0.5)	# a bit of delay to make sure it's not gonna immediately move

				###############################
				## keep getting last lat/lon ##
				###############################
				lat_last = self.lat
				lon_last = self.lon

				self.ch7_from_high = True
				#################################
				## keep drive as straight line ##
				#################################
				self.sbus_cmd.data = [self.sbus_steering, self.sbus_throttle]
				self.sbus_cmd_pub.publish(self.sbus_cmd)

			elif (self.calib_flag == False) and self.ch7_from_high:
				##################################################################
				## switch atcart_mode back to manual to make it completely stop ##
				## send False on hdg_calib_flag to let autopilot node knows     ##
				##################################################################
				self.atcart_mode_cmd_msg.data = 1
				self.atcart_mode_cmd_pub.publish(self.atcart_mode_cmd_msg)
				time.sleep(0.5) # a bit of delay makes other autopilot node not freak out
				self.hdg_calib_flag_msg.data = False
				self.hdg_calib_flag_pub.publish(self.hdg_calib_flag_msg)

				self.sbus_cmd.data = [1024, 1024]
				self.sbus_cmd_pub.publish(self.sbus_cmd)


				self.ch7_from_high = False
				line_hdg = self.get_bearing(lat_start, lon_start, lat_last, lon_last)
				self.hdg_offset = self.start_hdg - self.ConvertTo360Range(line_hdg)
				rospy.loginfo("heading offset {:.2f}".format(self.hdg_offset))


			else:
				self.get_latlon_once = True
				self.ch7_from_high = False


			# print("roll: {:.2f} | pitch: {:.2f} | heading: {:.2f}".format(self.roll, self.pitch, hdg))

			self.compass_msg.data = [self.roll, self.pitch, hdg]
			self.compass_pub.publish(self.compass_msg)



			rate.sleep()


if __name__ == '__main__':
	a = Imu2Compass()
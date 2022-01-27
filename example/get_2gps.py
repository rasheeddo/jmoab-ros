#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray
import numpy as np

class TwoGPS:

	def __init__(self):

		rospy.init_node('jmoab_ros_imu_to_compass', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-IMU_COMPASS node")

		# self.compass_pub = rospy.Publisher('/jmoab_compass', Float32MultiArray, queue_size=10)
		# self.compass_msg = Float32MultiArray()

		self.lat1 = 0.0
		self.lon1 = 0.0
		self.fix_stat1 = 0

		self.lat2 = 0.0
		self.lon2 = 0.0
		self.fix_stat2 = 0

		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/ublox2/fix", NavSatFix, self.gps2_callback)

		self.loop()

		rospy.spin()

	def gps_callback(self, msg):

		self.lat1 = msg.latitude
		self.lon1 = msg.longitude

		self.fix_stat1 = msg.status.status

	def gps2_callback(self, msg):
		self.lat2 = msg.latitude
		self.lon2 = msg.longitude

		self.fix_stat2 = msg.status.status

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

	def loop(self):

		rate = rospy.Rate(5)

		while not rospy.is_shutdown():

			if (self.lat1 != 0) and (self.lat2 != 0) and (self.fix_stat1 == 2) and (self.fix_stat2 == 2):

				brg = self.get_bearing(self.lat1, self.lon1, self.lat2, self.lon2)

				print(brg)

			else:
				print("fix1: {:d} | fix2: {:d}".format(self.fix_stat1, self.fix_stat2))

			rate.sleep()



if __name__ == '__main__':
	a = TwoGPS()
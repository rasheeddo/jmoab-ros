
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray, Int8, Int32MultiArray
import numpy as np
import matplotlib.pyplot as plt

class PlotGPS:

	def __init__(self):

		rospy.init_node('apm_planner_node', anonymous=True)

		self.get_gps1 = False
		self.get_gps2 = False

		self.lat_list1 = np.array([])
		self.lon_list1 = np.array([])
		self.lat_list2 = np.array([])
		self.lon_list2 = np.array([])

		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/ublox2/fix", NavSatFix, self.gps2_callback)

		self.loop()

		rospy.spin()

	def gps_callback(self, msg):

		self.lat1 = msg.latitude
		self.lon1 = msg.longitude

		self.get_gps1 = True

	def gps2_callback(self, msg):

		self.lat2 = msg.latitude
		self.lon2 = msg.longitude

		self.get_gps2 = True


	def loop(self):

		rate = rospy.Rate(20) # 10hz

		while not rospy.is_shutdown():

			if self.get_gps1 and self.get_gps2:

				if len(self.lat_list1) < 5:
					self.lat_list1 = np.append(self.lat_list1, self.lat1)
					self.lon_list1 = np.append(self.lon_list1, self.lon1)
					self.lat_list2 = np.append(self.lat_list2, self.lat2)
					self.lon_list2 = np.append(self.lon_list2, self.lon2)
				else:
					self.lat_list1 = np.delete(self.lat_list1, 0)
					self.lon_list1 = np.delete(self.lon_list1, 0)

					self.lat_list1 = np.append(self.lat_list1, self.lat1)
					self.lon_list1 = np.append(self.lon_list1, self.lon1)

					self.lat_list2 = np.delete(self.lat_list2, 0)
					self.lon_list2 = np.delete(self.lon_list2, 0)

					self.lat_list2 = np.append(self.lat_list2, self.lat2)
					self.lon_list2 = np.append(self.lon_list2, self.lon2)

				scat1 = plt.scatter(self.lon_list1, self.lat_list1, c='b')
				scat2 = plt.scatter(self.lon_list2, self.lat_list2, c='r')

				plt.pause(0.5)
				scat1.remove()
				scat2.remove()

			rate.sleep()

		plt.show()

if __name__ == "__main__":

	a = PlotGPS()
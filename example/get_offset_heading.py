import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import numpy as np

class GetOffsetHdg:

	def __init__ (self):
		rospy.init_node('apm_planner_node', anonymous=True)
		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/jmoab_compass", Float32MultiArray, self.compass_callback)
		rospy.Subscriber("/sbus_rc_ch", Int32MultiArray, self.sbus_callback)

		self.lat = 0.0
		self.lon = 0.0
		self.hdg = 0.0

		self.record = False

		self.loop()
		rospy.spin()


	def gps_callback(self, msg):

		self.lat = msg.latitude
		self.lon = msg.longitude
		
	def compass_callback(self, msg):

		self.hdg = msg.data[2]

	def sbus_callback(self, msg):
		if msg.data[8] > 1000 and self.record == False:
			self.record = True

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

	def ConvertTo360Range(self, deg):

		if deg < 0.0:
			deg = deg%360.0

		return deg


	def loop(self):

		rate = rospy.Rate(50)
		lat_list = []
		lon_list = []
		while not rospy.is_shutdown():


			if self.lat != 0.0 and self.lon != 0.0:
				
				if len(lat_list) < 100:
					print("Don't move the bot!")
					lat_list.append(self.lat)
					lon_list.append(self.lon)

				else:
					print("Move the bot as straight line for 5 meters, if finish trigger RC ch. to calculate heading offset")


					if self.record == True:
						lat_start = sum(lat_list)/float(len(lat_list))
						lon_start = sum(lon_list)/float(len(lon_list))

						lat_last = self.lat
						lon_last = self.lon

						line_hdg = self.get_bearing(lat_start, lon_start, lat_last, lon_last)

						offset_hdg = self.hdg - self.ConvertTo360Range(line_hdg)

						print(" ")
						print("compass_hdg: {:.3f} | line_hdg: {:.3f} | line_hdg_360: {:.3f} | offset_hdg: {:.3f}".format(self.hdg, line_hdg, self.ConvertTo360Range(line_hdg), offset_hdg))
						print(" ")
						print("write offset_hdg to heading_offset.txt")
						print("You will need to reload jmoab-compass node to take effect")
						with open("heading_offset.txt", "w+") as f:
							f.write(str(offset_hdg))

						quit()



			rate.sleep()


if __name__ == "__main__":

	g = GetOffsetHdg()
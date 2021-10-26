#! /usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import time
import numpy as np

class JMOAB_ADC:

	def __init__(self):
		rospy.init_node('jmoab_ros_adc_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ADC-Simulation node")

		self.adc_pub = rospy.Publisher("/jmoab_adc", Float32MultiArray, queue_size=10)
		self.adc_array = Float32MultiArray()

		rospy.loginfo("Publishing ADC values on /jmoab_adc topic")

		self.start_time = time.time()
		self.battery_life = 21600.0 # in seconds, 60x60xhours
		self.battery_full_charged = 25.2 # in volt, 6 cells LIPO
		self.battery_lower_limit = 18.0
		self.battery_range = self.battery_full_charged - self.battery_lower_limit
		self.time_spending = 0.0

		self.loop()

		rospy.spin()

	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		return out

	def convert2voltage(self, raw_list):
	
		raw_list = np.asarray(raw_list)
		voltage_array =  self.map(raw_list, 0.0, 255.0, 0.0, 40.96)
		return voltage_array

	def loop(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			self.time_spending = time.time() - self.start_time
			percent_usage = (100.0*self.time_spending/self.battery_life)

			battery_usage = (self.battery_range) - (self.battery_range)*((100.0 - percent_usage)/100.0)

			battery_remaining = self.battery_full_charged - battery_usage

			# print("percent_usage: {:.4f} | battery_usage: {:} | battery_remaining: {:}".format(percent_usage, battery_usage, battery_remaining))
		
			self.adc_array.data = [battery_remaining, 0.0, 0.0, 0.0, 0.0, 0.0]
			self.adc_pub.publish(self.adc_array)

			if battery_remaining < self.battery_lower_limit:
				rospy.loginfo("Battery is empty, usage time is {:.3f} seconds".format(self.time_spending))
				rospy.loginfo("Please restart jmoab-ros-adc-simulation to make battery fully charged again")
				quit()

			rate.sleep()

if __name__ == "__main__":

	ADC = JMOAB_ADC()
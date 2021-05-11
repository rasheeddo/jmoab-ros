#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Float32MultiArray
import time
import numpy as np

class JMOAB_ADC:

	def __init__(self):
		rospy.init_node('jmoab_ros_adc_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ADC node")

		self.bus = SMBus(1)

		self.adc_pub = rospy.Publisher("/jmoab_adc", Float32MultiArray, queue_size=10)
		self.adc_array = Float32MultiArray()

		rospy.loginfo("Publishing ADC values on /jmoab_adc topic")

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

			raw = self.bus.read_i2c_block_data(0x71, 0x00, 6)
			voltage_list = self.convert2voltage(raw)
			self.adc_array.data = voltage_list 
			self.adc_pub.publish(self.adc_array)

			rate.sleep()

if __name__ == "__main__":

	ADC = JMOAB_ADC()
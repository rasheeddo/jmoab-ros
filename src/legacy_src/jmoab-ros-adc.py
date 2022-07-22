#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Float32MultiArray
import time
import numpy as np
import argparse

class JMOAB_ADC(object):

	def __init__(self, NS):
		rospy.init_node('jmoab_ros_adc_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ADC node")

		self.bus = SMBus(1)

		adc_topic = self.namespace_attaching(NS,"/jmoab_adc")
		self.adc_pub = rospy.Publisher(adc_topic, Float32MultiArray, queue_size=10)
		self.adc_array = Float32MultiArray()

		rospy.loginfo("Publishing ADC values on {:} topic".format(adc_topic))

		self.loop()

		rospy.spin()

	def namespace_attaching(self, NS, topic_name):
		if NS is None:
			return topic_name
		else:
			if NS.startswith("/"):
				topic_name = NS + topic_name
			else:
				topic_name = "/" + NS + topic_name
			return topic_name


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

	parser = argparse.ArgumentParser(description='ADC node of jmoab-ros')
	parser.add_argument('--ns',
						help="a namespace in front of original topic")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	ns = args.ns

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	ADC = JMOAB_ADC(ns)
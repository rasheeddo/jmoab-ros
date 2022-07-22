#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Bool
import time

class JMOAB_Relay:

	def __init__(self):

		rospy.init_node('jmoab_ros_relay_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-Relay node")

		self.bus = SMBus(1)

		rospy.Subscriber("/jmoab_relay1", Bool, self.relay1_callback)
		rospy.Subscriber("/jmoab_relay2", Bool, self.relay2_callback)

		self.relay1_state = False
		self.relay2_state = False

		self.prev_relay1_state = self.relay1_state
		self.prev_relay2_state = self.relay2_state


		rospy.loginfo("Subscribing on /jmoab_relay1 /jmoab_relay2 topics for relays control")

		# self.loop()

		rospy.spin()


	def set_relay(self, _id, value):

		self.bus.write_byte_data(0x70, _id, value)


	def relay1_callback(self, msg):
		self.relay1_state = msg.data
		self.set_relay(0, self.relay1_state)

	def relay2_callback(self, msg):
		self.relay2_state = msg.data
		self.set_relay(1, self.relay2_state)
			


	def loop(self):

		rate = rospy.Rate(20) # 10hz

		while not rospy.is_shutdown():

			# if self.prev_relay1_state != self.relay1_state:
			self.set_relay(0, self.relay1_state)

			# if self.prev_relay2_state != self.relay2_state:
			self.set_relay(1, self.relay2_state)


			self.prev_relay1_state = self.relay1_state
			self.prev_relay2_state = self.relay2_state

			rate.sleep()


if __name__ == '__main__':
	jmoab = JMOAB_Relay()
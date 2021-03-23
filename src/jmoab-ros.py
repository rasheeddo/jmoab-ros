#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray

class JMOAB:

	def __init__(self):

		rospy.init_node('jmoab_ros_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS node")

		self.bus = SMBus(1)

		self.sbus_ch_pub = rospy.Publisher("/sbus_rc_ch", Int32MultiArray, queue_size=10)
		self.sbus_ch = Int32MultiArray()

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.cmd_callback)
		self.cmd_steering = 1024
		self.cmd_throttle = 1024

		rospy.loginfo("Publishing SBUS RC channel on /sbus_rc_ch topic")
		rospy.loginfo("Subscribing on /sbus_cmd topic for steering and throttle values")

		self.loop()

		rospy.spin()


	def sbus2word(self, sbus_val):

		high_byte = sbus_val >> 8
		low_byte = (sbus_val & 0x00FF)

		return [high_byte, low_byte]

	def send_steering_throttle(self, sbus_steering, sbus_throttle):

		steering_bytes = self.sbus2word(sbus_steering)
		throttle_bytes = self.sbus2word(sbus_throttle)

		## combine as 4 elements [str_H, str_L, thr_H, thr_L]
		all_bytes = steering_bytes+throttle_bytes

		self.bus.write_i2c_block_data(0x71, 0x30, all_bytes)

	def get_sbus_channel(self):
		input_SBUS = self.bus.read_i2c_block_data(0x71, 0x0C, 32)

		SBUS_ch = [None]*16
		for i in range(16):
			SBUS_ch[i] = (input_SBUS[(2*i)+1] & 0xFF) | ((input_SBUS[i*2] & 0xFF) << 8)

		return SBUS_ch

	def cmd_callback(self, msg):
		self.cmd_steering = msg.data[0]
		self.cmd_throttle = msg.data[1]

		self.send_steering_throttle(self.cmd_steering, self.cmd_throttle)

	def loop(self):

		rate = rospy.Rate(100) # 10hz

		while not rospy.is_shutdown():
			
			sbus_ch_array = self.get_sbus_channel()
			self.sbus_ch.data = sbus_ch_array
			self.sbus_ch_pub.publish(self.sbus_ch)

			rate.sleep()


if __name__ == '__main__':
	jmoab = JMOAB()
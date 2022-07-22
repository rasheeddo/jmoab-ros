#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from std_msgs.msg import Int32MultiArray
import time

class JMOAB_Servo:

	def __init__(self):

		rospy.init_node('jmoab_ros_servo_control_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-Servo-Control node")

		self.bus = SMBus(1)

		rospy.Subscriber("/jmoab_servos", Int32MultiArray, self.servo_cmd_callback)

		self.pwm_min = 1120
		self.pwm_max = 1920
		self.pwm_mid = 1520


		rospy.loginfo("Subscribing on /jmoab_servos topic for servos control")

		# self.loop()

		rospy.spin()


	def pwm2word(self, pwm_val):

		high_byte = pwm_val >> 8
		low_byte = (pwm_val & 0x00FF)

		return [high_byte, low_byte]

	def send_pwm_bytes(self, servo_id, pwm):

		pwm_bytes = self.pwm2word(pwm)

		if servo_id == 0:
			self.bus.write_i2c_block_data(0x71, 0x06, pwm_bytes)
		elif servo_id == 1:
			self.bus.write_i2c_block_data(0x71, 0x08, pwm_bytes)
		if servo_id == 2:
			self.bus.write_i2c_block_data(0x71, 0x0A, pwm_bytes)

	def over_limit_check(self, pwm):

		if pwm < self.pwm_min:
			pwm  = self.pwm_min
		elif pwm > self.pwm_max:
			pwm = self.pwm_max

		return pwm

	def servo_cmd_callback(self, msg):
		if len(msg.data) > 0 and len(msg.data) < 3:

			for servo_id in range(len(msg.data)):
				self.cmd_pwm = msg.data[servo_id]
				self.cmd_pwm = self.over_limit_check(self.cmd_pwm)
				self.send_pwm_bytes(servo_id, self.cmd_pwm)


	def map(self, val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == '__main__':
	jmoab = JMOAB_Servo()
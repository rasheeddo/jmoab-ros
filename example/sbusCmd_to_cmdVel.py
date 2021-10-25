import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class Vel2Sbus:

	def __init__(self):

		rospy.init_node('jmoab_ros_vel_2_sbus', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-SBUS_CMD-CMD_VEl node")

		rospy.Subscriber("/sbus_cmd", Int32MultiArray, self.sbus_cmd_callback)

		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()

		self.cmd_steering = 1024
		self.cmd_throttle = 1024

		self.sbus_mid = 1024
		self.sbus_min = 368
		self.sbus_max = 1680

		self.max_vel = 2.2	# m/s
		self.min_vel = -2.2

		self.max_ang_vel = 7.0	# rad/s
		self.min_ang_vel = -7.0

		## -143	 RPM -->  368
		## 4.5   RPM -->  1100
		## 143   RPM -->  1680


		# self.loop()

		rospy.spin()

	def sbus_cmd_callback(self, msg):

		self.cmd_steering = msg.data[0]
		self.cmd_throttle = msg.data[1]

		if self.cmd_throttle != 1024:
			vx = self.map(self.cmd_throttle, self.sbus_min, self.sbus_max, self.min_vel, self.max_vel)
		else:
			vx = 0.0

		if vx >= 0:
			if self.cmd_steering != 1024:
				wz = self.map(self.cmd_steering, self.sbus_min, self.sbus_max, self.max_ang_vel, self.min_ang_vel)
			else:
				wz = 0.0
		else:
			if self.cmd_steering != 1024:
				wz = self.map(self.cmd_steering, self.sbus_min, self.sbus_max, self.min_ang_vel, self.max_ang_vel)
			else:
				wz = 0


		self.cmd_vel.linear.x = vx
		self.cmd_vel.angular.z = wz
		self.cmd_vel_pub.publish(self.cmd_vel)

		print("str: {:d} | thr: {:d} | vx: {:.2f} | wz: {:.2f}".format(self.cmd_steering, self.cmd_throttle, vx, wz))


	def map(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter

		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		return out


	# def loop(self):

	# 	rate = rospy.Rate(20)


	# 	while not rospy.is_shutdown():




	# 		rate.sleep()

if __name__ == "__main__":

	v2s = Vel2Sbus()

#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Int32MultiArray, Float32MultiArray, Int8
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import struct
import numpy as np
import sys
import serial
import json

class JMOAB_ICM20948_DMP(object):

	def __init__(self, ser_port):

		rospy.init_node('jmoab_ros_icmp20948_dmp', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-ICM20948-DMP node")

		rospy.loginfo("Publishing /imu/data topic")

		self.ser = serial.Serial(ser_port, 115200)

		self.br = tf2_ros.TransformBroadcaster()
		self.t = TransformStamped()

		self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
		self.imu_msg = Imu()


		self.loop()

		rospy.spin()
	

	def loop(self):
		rate = rospy.Rate(20) # 10hz

		while not rospy.is_shutdown():
			startTime = time.time()


			try:
				line = self.ser.readline()
				string_data = line.decode().strip()
				load_data = json.loads(string_data)
				q0 = load_data["q0"]
				q1 = load_data["q1"]
				q2 = load_data["q2"]
				q3 = load_data["q3"]

				print("q0 {:.3f} q1 {:.3f} q2 {:.3f} q3 {:.3f}".format(q0,q1,q2,q3))

				self.imu_msg.header.stamp = rospy.Time.now()
				self.imu_msg.header.frame_id = "imu_link"
				self.imu_msg.orientation.x = q1 #roll
				self.imu_msg.orientation.y = q2 #pitch
				self.imu_msg.orientation.z = q3 #yaw
				self.imu_msg.orientation.w = q0 
				self.imu_msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

				self.imu_pub.publish(self.imu_msg)

		

			except Exception as e:
				print(e)
				print("Failed to parse")
				pass

			rate.sleep()

if __name__ == "__main__":

	if len(sys.argv) < 2:
		print("Please specify Arduino serial device port")
		print("usage: rosrun jmoab-ros icm20948_dmp_json.py /dev/ttyACM0")
	else:
		ser_port = str(sys.argv[1])
		jhw = JMOAB_ICM20948_DMP(ser_port)

	
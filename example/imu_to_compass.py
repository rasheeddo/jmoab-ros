#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np

class Imu2Compass:

	def __init__(self):

		rospy.init_node('jmoab_ros_imu_to_compass', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-IMU_COMPASS node")

		# rospy.Subscriber("/imu", Imu, self.imu_callback)
		rospy.Subscriber("/odom", Odometry, self.odom_callback)

		self.compass_pub = rospy.Publisher('/jmoab_compass', Float32MultiArray, queue_size=10)
		self.compass_msg = Float32MultiArray()

		rospy.spin()

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg

	# def imu_callback(self,data):

	# 	qw = data.orientation.w
	# 	qx = data.orientation.x
	# 	qy = data.orientation.y
	# 	qz = data.orientation.z

	# 	r11 = qw**2 + qx**2 - qy**2 - qz**2 #1 - 2*qy**2 - 2*qz**2
	# 	r12 = 2*qx*qy - 2*qz*qw
	# 	r13 = 2*qx*qz + 2*qy*qw
	# 	r21 = 2*qx*qy + 2*qz*qw
	# 	r22 = qw**2 - qx**2 + qy**2 - qz**2	#1 - 2*qx**2 - 2*qz**2
	# 	r23 = 2*qy*qz - 2*qx*qw
	# 	r31 = 2*qx*qz - 2*qy*qw
	# 	r32 = 2*qy*qz + 2*qx*qw
	# 	r33 = qw**2 - qx**2 - qy**2 + qz**2	#1 - 2*qx**2 - 2*qy**2
	# 	rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

	# 	## default orientation (heatsink is pointing down)
	# 	rot_matrix = np.array([
	# 							[ 1, 0, 0],
	# 							[ 0, 1, 0],
	# 							[ 0, 0, 1]])

	# 	## board is pitch 180 (heatsink is pointing up)
	# 	# rot_matrix = np.array([
	# 	# 						[-1, 0, 0],
	# 	# 						[ 0, 1, 0],
	# 	# 						[ 0, 0,-1]])

	# 	rot = np.dot(rot,rot_matrix)

	# 	heading = self.ConvertTo360Range(np.degrees(np.arctan2(rot[1,0],rot[0,0])))
	# 	pitch = np.degrees(np.arcsin(-rot[2,0]))
	# 	roll = np.degrees(np.arctan2(rot[2,1],rot[2,2]))

	# 	print("roll: {:.2f} | pitch: {:.2f} | heading: {:.2f}".format(roll, pitch, heading))

	# 	self.compass_msg.data = [roll, pitch, heading]
	# 	self.compass_pub.publish(self.compass_msg)

	def odom_callback(self,data):

		qw = data.pose.pose.orientation.w
		qx = data.pose.pose.orientation.x
		qy = data.pose.pose.orientation.y
		qz = data.pose.pose.orientation.z

		r11 = qw**2 + qx**2 - qy**2 - qz**2 #1 - 2*qy**2 - 2*qz**2
		r12 = 2*qx*qy - 2*qz*qw
		r13 = 2*qx*qz + 2*qy*qw
		r21 = 2*qx*qy + 2*qz*qw
		r22 = qw**2 - qx**2 + qy**2 - qz**2	#1 - 2*qx**2 - 2*qz**2
		r23 = 2*qy*qz - 2*qx*qw
		r31 = 2*qx*qz - 2*qy*qw
		r32 = 2*qy*qz + 2*qx*qw
		r33 = qw**2 - qx**2 - qy**2 + qz**2	#1 - 2*qx**2 - 2*qy**2
		rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

		## default orientation (heatsink is pointing down)
		rot_matrix = np.array([
								[ 1, 0, 0],
								[ 0, 1, 0],
								[ 0, 0, 1]])

		## board is pitch 180 (heatsink is pointing up)
		# rot_matrix = np.array([
		# 						[-1, 0, 0],
		# 						[ 0, 1, 0],
		# 						[ 0, 0,-1]])

		rot = np.dot(rot,rot_matrix)

		heading = self.ConvertTo360Range(np.degrees(np.arctan2(rot[1,0],rot[0,0])))
		pitch = np.degrees(np.arcsin(-rot[2,0]))
		roll = np.degrees(np.arctan2(rot[2,1],rot[2,2]))

		print("roll: {:.2f} | pitch: {:.2f} | heading: {:.2f}".format(roll, pitch, heading))

		self.compass_msg.data = [roll, pitch, heading]
		self.compass_pub.publish(self.compass_msg)


if __name__ == '__main__':
	a = Imu2Compass()
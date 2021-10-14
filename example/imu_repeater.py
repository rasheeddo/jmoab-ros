#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('/jmoab_imu_repeat', Imu, queue_size=10)
_imu_msg = Imu()

def callback(data):

	# print(data)

	qw = data.orientation.w
	qx = data.orientation.x
	qy = data.orientation.y
	qz = data.orientation.z

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
	# rot_matrix = np.array([
	# 						[ 1, 0, 0],
	# 						[ 0, 1, 0],
	# 						[ 0, 0, 1]])

	## board is pitch 180 (heatsink is pointing up)
	rot_matrix = np.array([
							[-1, 0, 0],
							[ 0, 1, 0],
							[ 0, 0,-1]])

	rot = np.dot(rot,rot_matrix)
	y = np.arctan2(rot[1,0],rot[0,0])
	p = np.arcsin(-rot[2,0])
	r = np.arctan2(rot[2,1],rot[2,2])

	QW = np.cos(r/2)*np.cos(p/2)*np.cos(y/2) + np.sin(r/2)*np.sin(p/2)*np.sin(y/2)
	QX = np.sin(r/2)*np.cos(p/2)*np.cos(y/2) - np.cos(r/2)*np.sin(p/2)*np.sin(y/2)
	QY = np.cos(r/2)*np.sin(p/2)*np.cos(y/2) + np.sin(r/2)*np.cos(p/2)*np.sin(y/2)
	QZ = np.cos(r/2)*np.cos(p/2)*np.sin(y/2) - np.sin(r/2)*np.sin(p/2)*np.cos(y/2)

	_imu_msg.header.stamp = rospy.Time.now()
	_imu_msg.header.frame_id = 'base_link'
	_imu_msg.orientation.x = QX #roll
	_imu_msg.orientation.y = QY #pitch
	_imu_msg.orientation.z = QZ #yaw
	_imu_msg.orientation.w = QW 

	pub.publish(_imu_msg)


	print("roll %.2f pitch %.2f yaw2 %.2f" %(np.degrees(r),np.degrees(p),np.degrees(y)))


def listener():

	rospy.init_node('jmoab_imu_listener_node', anonymous=True)
	rospy.Subscriber("/jmoab_imu_raw", Imu, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()
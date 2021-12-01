#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros

br = tf2_ros.TransformBroadcaster()
t = TransformStamped()
ekf_odom_pub = rospy.Publisher("/ekf_odom", PoseWithCovarianceStamped, queue_size=10)
ekf_odom_msg = PoseWithCovarianceStamped()

def callback(data):

	# print(data)

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z

	qw = data.pose.pose.orientation.w
	qx = data.pose.pose.orientation.x
	qy = data.pose.pose.orientation.y
	qz = data.pose.pose.orientation.z

	# construct tf
	t.header.frame_id = "ekf_odom" 
	t.header.stamp = rospy.Time.now()
	t.child_frame_id = "base_link_ekf"	#"base_footprint"	#"base_link"
	t.transform.translation.x = x
	t.transform.translation.y = y
	t.transform.translation.z = z

	t.transform.rotation.x = qx
	t.transform.rotation.y = qy
	t.transform.rotation.z = qz
	t.transform.rotation.w = qw
	br.sendTransform(t)

	ekf_odom_msg.header.stamp = rospy.Time.now()
	ekf_odom_msg.header.frame_id = "ekf_odom"
	ekf_odom_msg.pose.pose.position.x = x
	ekf_odom_msg.pose.pose.position.y = y
	ekf_odom_msg.pose.pose.position.z = z
	ekf_odom_msg.pose.pose.orientation.x = qx
	ekf_odom_msg.pose.pose.orientation.y = qy
	ekf_odom_msg.pose.pose.orientation.z = qz
	ekf_odom_msg.pose.pose.orientation.w = qw

	ekf_odom_msg.pose.covariance = data.pose.covariance

	ekf_odom_pub.publish(ekf_odom_msg)


def listener():

	rospy.init_node('generate_tf_for_ekf_odom', anonymous=True)
	rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()
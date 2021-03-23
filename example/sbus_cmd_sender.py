#! /usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def set_steering_throttle():
	rospy.init_node('sbus_cmd_sender_node', anonymous=True)
	pub = rospy.Publisher('/sbus_cmd', Int32MultiArray, queue_size=10)
	steering_throttle_array = Int32MultiArray()

	rate = rospy.Rate(50) # 50hz

	while not rospy.is_shutdown():
		## sbus steering and throttle values are ranging from min. 368 to max. 1680
		## sbus middle value is 1024, so when steering and throttle are 1024, the UGV won't move
		## when steering to the left, sbus_steering is > 1024
		## when steering to the right, sbus_steering is < 1024
		## when throttle forward, sbus_throttle is > 1024
		## when throttle backward, sbus_throttle is < 1024

		## WARNING! 
		## you are working with 7km/h UGV, full throttle can cause damage to object and human surrounding,
		## so DON'T randomly put numbers below if you're not sure what you're doing.
		sbus_steering = 1024 # 1024 is no steering
		sbus_throttle = 1100 # 1100 is quite low speed

		steering_throttle_array.data = [sbus_steering, sbus_throttle]
		pub.publish(steering_throttle_array)
		rate.sleep()

if __name__ == '__main__':
	try:
		set_steering_throttle()
	except rospy.ROSInterruptException:
		pass
#! /usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def callback(data):
	## this will print of 16 channels of SBUS
	## even your transmitter has less channels that 16, it's also stored as what you have.
	## ch 1 : steering (min: 368, mid: 1024, max: 1680)
	## ch 2 : throttle (min: 368, mid: 1024, max: 1680)
	## ch 5 : mode changing [144 (high) is hold-mode, 1024 (mid) is manual-mode, 1904 (low) is auto-mode]
	## ch 6 : joystick mode [144 (high) is regular RC control, 1904 (low) is joystick control], most of the time we set as 144 (high)
	## you can use other switches and sticks on the transmitter for other purpose, but above channels are reserved for UGV control
	print(data.data)

def listener():

	rospy.init_node('sbus_ch_listener_node', anonymous=True)
	rospy.Subscriber("/sbus_rc_ch", Int32MultiArray, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()
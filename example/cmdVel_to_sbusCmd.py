import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class Vel2Sbus:

	def __init__(self):

		rospy.init_node('jmoab_ros_vel_2_sbus', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-CMD_VEL-SBUS_CMD node")

		rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)

		self.sbus_cmd_pub = rospy.Publisher('/sbus_cmd', Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()

		self.rpm_L = 0.0
		self.rpm_R = 0.0

		self.Vx = 0.0
		self.Wz = 0.0

		self.R_wheel = 0.15		# 15cm wheel's radius
		self.L_cart = 0.46		# 46cm from left to right wheel
		##### SBUS Parameters #####
		self.sbus_mid = 1024
		self.sbus_steering_mid = self.sbus_mid
		self.sbus_throttle_mid = self.sbus_mid

		self.sbus_skidding_left_min = 940
		self.sbus_skidding_right_min = 1100

		self.sbus_steering_left_min = 966	#940
		self.sbus_steering_right_min = 1074	#1100

		self.sbus_throttle_fwd_min = 1090
		self.sbus_throttle_bwd_min = 970

		self.MIN_SBUS = 368
		self.MAX_SBUS = 1680

		self.MAX_RPM = 143.0

		self.MIN_STEER_FWD_RPM = 25.0
		self.MIN_STEER_BWD_RPM = 28.0 

		self.Vx_max = 4.4
		self.Wz_max = 9.7

		## -143	 RPM -->  368
		## 4.5   RPM -->  1100
		## 143   RPM -->  1680
		##### PID Parameters #####
		# self.kp_L


		self.loop()

		rospy.spin()

	def vel_callback(self, msg):

		self.Vx = msg.linear.x
		self.Wz = msg.angular.z


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

	# def RPM2SBUS_Throttle(self,rpm):

	# 	if rpm == 0.0:
	# 		sbus = self.sbus_mid
	# 	else:
	# 		if rpm > 0.0:
	# 			sbus = self.map(rpm, 0.0, self.MAX_RPM, self.sbus_throttle_fwd_min, self.MAX_SBUS)
	# 		else:
	# 			sbus = self.map(rpm, -self.MAX_RPM, 0.0, self.MIN_SBUS, self.sbus_throttle_bwd_min)

	# 	return int(sbus)

	# def RPM2SBUS_Skidding(self,rpm):

	# 	if rpm == 0.0:
	# 		sbus = self.sbus_mid
	# 	else:
	# 		if rpm > 0.0:
	# 			sbus = self.map(rpm, 0.0, self.MAX_RPM, self.sbus_skidding_right_min, self.MAX_SBUS)
	# 		else:
	# 			sbus = self.map(rpm, -self.MAX_RPM, 0.0, self.MIN_SBUS, self.sbus_skidding_left_min)

	# 	return int(sbus)

	# def RPM2SBUS_Steering(self, rpm):
	# 	if rpm > 0.0:
	# 		sbus = self.map(rpm, self.MIN_STEER_FWD_RPM, self.MAX_RPM, self.MAX_SBUS, self.sbus_steering_mid)
	# 	else:
	# 		sbus = self.map(rpm, -self.MAX_RPM, self.MIN_STEER_BWD_RPM, self.sbus_steering_mid, self.MAX_SBUS)

	# def RPM2SBUS_Mixing(self, rpm_L, rpm_R):

	# 	# print(rpm_L, rpm_R)
	# 	## upper quadrant 2 or 1
	# 	if (rpm_L > 0.0) and (rpm_R > 0.0):
	# 		## curve to right, quadrant 1
	# 		if (rpm_L > rpm_R):
	# 			print("1st")
	# 			sbus_throttle = self.RPM2SBUS_Throttle(rpm_L)
	# 			sbus_steering = self.map(rpm_R, 0.0, rpm_L, self.MAX_SBUS, self.sbus_steering_mid)
			
	# 		## curve to left, quadrant 2
	# 		else:
	# 			print("2nd")
	# 			sbus_throttle = self.RPM2SBUS_Throttle(rpm_R)
	# 			sbus_steering = self.map(rpm_L, 0.0, rpm_R, self.MIN_SBUS, self.sbus_steering_mid)

	# 	## lower quadrant 3 or 4
	# 	elif (rpm_L < 0.0) and (rpm_R < 0.0):
	# 		## curve to left backward, quadrant4, rpm_L more negative than rpm_R
	# 		if (rpm_L < rpm_R):
	# 			print("4th")
	# 			sbus_throttle = self.RPM2SBUS_Throttle(rpm_L)
	# 			sbus_steering = self.map(rpm_R, rpm_L, 0.0, self.sbus_steering_mid, self.MIN_SBUS)
			
	# 		## curve to right backward, quadrant3, rpm_R more negative than rpm_L
	# 		else:
	# 			print("3rd")
	# 			sbus_throttle = self.RPM2SBUS_Throttle(rpm_R)
	# 			sbus_steering = self.map(rpm_L, rpm_R, 0.0, self.sbus_steering_mid, self.MAX_SBUS)

	# 	## this is likely skidding...
	# 	elif (rpm_L > 0.0) and (rpm_R < 0.0):
	# 		print("likely skidding right...")
	# 		sbus_throttle = self.RPM2SBUS_Throttle(rpm_L)
	# 		sbus_steering = self.RPM2SBUS_Skidding(rpm_R)

	# 	elif (rpm_L < 0.0) and (rpm_R > 0.0):
	# 		print("likely skidding left...")
	# 		sbus_throttle = self.RPM2SBUS_Throttle(rpm_R)
	# 		sbus_steering = self.RPM2SBUS_Skidding(rpm_Lsbus_steering_mid)

	# 	return int(sbus_steering), int(sbus_throttle)

	def Vel2SBUS_Mixing(self, vx, wz):

		sbus_steering = self.Vel2SBUS_Steering(wz)
		sbus_throttle = self.Vel2SBUS_Throttle(vx)

		return int(sbus_steering), int(sbus_throttle)

	def Vel2SBUS_Throttle(self, vx):

		if vx > 0.0:
			sbus_throttle = self.map(vx, 0.0, self.Vx_max, self.sbus_throttle_fwd_min, self.MAX_SBUS)
		else:
			sbus_throttle = self.map(vx, -self.Vx_max, 0.0, self.MIN_SBUS, self.sbus_throttle_bwd_min)

		return int(sbus_throttle)

	def Vel2SBUS_Skidding(self, wz):

		if wz > 0.0:
			sbus_steering = self.map(wz, 0.0, self.Wz_max, self.sbus_skidding_left_min, self.MIN_SBUS)
		else:
			sbus_steering = self.map(wz, -self.Wz_max, 0.0, self.MAX_SBUS, self.sbus_skidding_right_min)

		return int(sbus_steering)

	def Vel2SBUS_Steering(self, wz):

		if wz > 0.0:
			sbus_steering = self.map(wz, 0.0, self.Wz_max, self.sbus_steering_left_min, self.MIN_SBUS)
		else:
			sbus_steering = self.map(wz, -self.Wz_max, 0.0, self.MAX_SBUS, self.sbus_steering_right_min)

		return int(sbus_steering)

	def Linear2RPM(self, vx):
		rpm = (vx/2.0)*(180.0/(self.R_wheel*6.0*np.pi))

		return rpm

	def Angular2RPM(self, wz):
		rpm = -wz*(self.L_cart/2.0)*(180.0/(self.R_wheel*6.0*np.pi))

		return rpm

	def loop(self):

		rate = rospy.Rate(20)


		while not rospy.is_shutdown():

			## curving
			if (self.Vx != 0.0) and (self.Wz != 0.0):
				R_icc = abs(self.Vx)/abs(self.Wz)
				sign_vx = self.Vx/abs(self.Vx)

				if self.Wz > 0.0:
					VL = (sign_vx)*(self.Wz*(R_icc - self.L_cart/2.0))/2.0
					VR = (sign_vx)*(self.Wz*(R_icc + self.L_cart/2.0))/2.0

					rpm_L = self.Linear2RPM(VL)
					rpm_R = self.Linear2RPM(VR)

				else:
					VL = (sign_vx)*(abs(self.Wz)*(R_icc + self.L_cart/2.0))/2.0
					VR = (sign_vx)*(abs(self.Wz)*(R_icc - self.L_cart/2.0))/2.0

					rpm_L = self.Linear2RPM(VL)
					rpm_R = self.Linear2RPM(VR)

				sbus_steering, sbus_throttle = self.Vel2SBUS_Mixing(self.Vx, self.Wz)

			## Go straight foward or backward
			elif (self.Vx != 0.0) and (self.Wz == 0.0):
				rpm = self.Linear2RPM(self.Vx)
				rpm_L = rpm
				rpm_R = rpm

				sbus_throttle = self.Vel2SBUS_Throttle(self.Vx)
				sbus_steering = self.sbus_mid

			## skidding in place
			elif (self.Vx == 0.0) and (self.Wz != 0.0):
				rpm = self.Angular2RPM(self.Wz)
				rpm_L = rpm
				rpm_R = -rpm

				sbus_throttle = self.sbus_mid
				sbus_steering = self.Vel2SBUS_Skidding(self.Wz)
			## No motion
			else:
				rpm_L = 0.0
				rpm_R = 0.0
				sbus_throttle = self.sbus_mid
				sbus_steering = self.sbus_mid
			

			print("vx: {:.3f} | wz: {:.3f}| rpm_L: {:.3f} | rpm_R: {:.3f} | str: {:d} | thr: {:d}".format(\
				self.Vx, self.Wz, rpm_L, rpm_R, sbus_steering, sbus_throttle))


			self.sbus_cmd.data = [sbus_steering, sbus_throttle]
			self.sbus_cmd_pub.publish(self.sbus_cmd)


			rate.sleep()

if __name__ == "__main__":

	v2s = Vel2Sbus()

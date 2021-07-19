from pymavlink import mavutil
import time
import argparse
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
import numpy as np


class APM(object):

	def __init__ (self, IP):
		rospy.init_node('apm_planner_node', anonymous=True)
		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/jmoab_compass", Float32MultiArray, self.compass_callback)

		self.lat = 0.0
		self.lon = 0.0
		self.alt = 0.0
		self.gps_fix = 0
		self.roll = 0.0
		self.pitch = 0.0
		self.hdg = 0.0

		mavutil.set_dialect("ardupilotmega")
		self.master = mavutil.mavlink_connection('udpout:{:}:14550'.format(IP))

		self.run()
		rospy.spin()


	def gps_callback(self, msg):

		self.lat = msg.latitude
		self.lon = msg.longitude
		self.alt = msg.altitude
		fix_stat = msg.status.status

		# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatStatus.html
		# https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
		if fix_stat == -1:
			## No fix
			self.gps_fix = 1

		elif fix_stat == 0:
			# On Ros, unaugmented fix?? I guess it's 3D fix on APM??
			self.gps_fix = 3

		elif fix_stat == 1:
			# On Ros, with satellite-based augmentation??? I guess it's DGPS on APM??
			self.gps_fix = 4

		elif fix_stat == 2:
			# On Ros, with ground-based augmentation??? I guess it's RTK-Fixed on APM??
			self.gps_fix = 6

	def compass_callback(self, msg):

		self.roll = msg.data[0]
		self.pitch = msg.data[1]
		self.hdg = msg.data[2]


	def run(self):


		rate = rospy.Rate(2) # 10hz

		while not rospy.is_shutdown():
			
			# time.sleep(0.1)
			Type = 10							# https://mavlink.io/en/messages/common.html#MAV_TYPE
			autopilot = 128						# Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. (type:uint8_t, values:MAV_AUTOPILOT)
			base_mode = (128 | 64 | 16 | 8 | 4)	# System mode bitmap. (type:uint8_t, values:MAV_MODE_FLAG)
			custom_mode = 10					# A bitfield for use for autopilot-specific flags (type:uint32_t)
			system_status = 4					# System status flag. (type:uint8_t, values:MAV_STATE), https://mavlink.io/en/messages/common.html#MAV_STATE
			mavlink_version = 3					# default is 3
			self.master.mav.heartbeat_send(Type, autopilot, base_mode, custom_mode, system_status, mavlink_version)


			# time.sleep(0.1)
			time_usec = int(time.time())	# Unix timestamp
			fix_type = self.gps_fix					# GPS fix type. (type:uint8_t, values:GPS_FIX_TYPE)
			lat = int(self.lat*1E7)			# Latitude (WGS84, EGM96 ellipsoid) [degE7] (type:int32_t)
			lon = int(self.lon*1E7)			# Longitude (WGS84, EGM96 ellipsoid) [degE7] (type:int32_t)
			alt = self.alt			# Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude. [mm] (type:int32_t)
			eph = 65535			# GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
			epv = 65535			# GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
			vel = 65535			# GPS ground speed. If unknown, set to: UINT16_MAX [cm/s] (type:uint16_t)
			cog = 65535			# Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX [cdeg] (type:uint16_t)
			sat_visible = 7	#Number of satellites visible. If unknown, set to 255 (type:uint8_t)
			self.master.mav.gps_raw_int_send(time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, sat_visible)


			# time.sleep(0.1)
			time_boot_ms = int(time.time()/1000.0)					# Timestamp (time since system boot). [ms] (type:uint32_t)
			roll = np.radians(self.roll)												# Roll angle (-pi..+pi) [rad] (type:float)
			pitch = np.radians(self.pitch)											# Pitch angle (-pi..+pi) [rad] (type:float)
			yaw = np.radians(self.hdg)												# Yaw angle (-pi..+pi) [rad] (type:float)
			rollspeed = 0.0											# Roll angular speed [rad/s] (type:float)
			pitchspeed = 0.0 										# Pitch angular speed [rad/s] (type:float)
			yawspeed = 0.0											# Yaw angular speed [rad/s] (type:float)
			self.master.mav.attitude_send(time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
			
			# time.sleep(0.1)
			# master.mav.sys_status_send(0, 0, 0, 0, 24, 0,0,0,0,0,0,0,0)

			print("lat: {:.6f} | lon: {:.6f} | hdg: {:.3f} | fix: {:d}".format(self.lat, self.lon, self.hdg, self.gps_fix))
			
			# time.sleep(1)
			rate.sleep()



if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='GPS visualizer on APM Planner2')
	parser.add_argument('--ip',
						help="This is your computer IP for APM Planner")

	args = parser.parse_args()
	ip = args.ip
	# FCU connection variables

	if ip is None:
		print("Error: please specify GCS computer IP (your PC)")
		quit()

	apm_planner = APM(ip)
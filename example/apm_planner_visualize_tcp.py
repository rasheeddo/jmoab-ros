from pymavlink import mavutil
import time
import argparse
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray, Int8, Int32MultiArray
from jmoab_autopilot_ros.msg import GoalWaypoints
import numpy as np
import os
import sys

python_version = sys.version_info[0]


class APM(object):

	def __init__ (self, IP, mission_dir, ID, NS):
		rospy.init_node('apm_planner_node', anonymous=True)

		## Initial value ##
		self.lat = 0.0
		self.lon = 0.0
		self.alt = 0.0
		self.gps_fix = 0
		self.roll = 0.0
		self.pitch = 0.0
		self.hdg = 0.0
		self.cart_mode = 0

		self.rc1 = 1500
		self.rc2 = 1500
		self.rc3 = 1500
		self.rc4 = 1500
		self.rc5 = 1500
		self.rc6 = 1500
		self.rc7 = 1500
		self.rc8 = 1500
		self.rc9 = 1500
		self.rc10 = 1500

		self.rc11 = 1500
		self.rc12 = 1500
		self.rc13 = 1500
		self.rc14 = 1500
		self.rc15 = 1500
		self.rc16 = 1500

		## Data rate timer ##
		self.attitude_rate = 4.0
		self.heartbeat_rate = 1.0
		self.sys_status_rate = 2.0
		self.gps_raw_int_rate = 2.0
		self.param_rate = 0.3 
		self.sys_time_rate = 2.0
		self.ahrs_rate = 2.0
		self.ahrs2_rate = 4.0
		self.print_rate = 1.0
		self.rcChannels_rate = 2.0

		self.hearbeat_last_send_stamp = time.time()
		self.gpsRawInt_last_send_stamp = time.time()
		self.attitude_last_send_stamp = time.time()
		self.sysStatus_last_send_stamp = time.time()
		self.param_last_send_stamp = time.time()
		self.sysTime_last_send_stamp = time.time()
		self.ahrs_last_send_stamp = time.time()
		self.ahrs2_last_send_stamp = time.time()
		self.print_last_stamp  = time.time()
		self.time_boot_ms_start = time.time()
		self.rcChannels_last_send_stamp = time.time()

		mavutil.set_dialect("ardupilotmega")
		_port = 5660 + (ID*100)
		self.master = mavutil.mavlink_connection('tcp:{:}:{:d}'.format(IP,_port), source_system=ID, source_component=1)
		print("Opening APMPlanner2 with TCP connection as server by port {:d}".format(_port))

		### Mission file ###
		self.mission_file_path = mission_dir

		if NS is None:
			gps_topic = "/ublox/fix"
			compass_topic = "/jmoab_compass"
		else:
			if NS.startswith("/"):
				gps_topic = NS + "/ublox/fix"
				compass_topic = NS + "/jmoab_compass"
			else:
				gps_topic = "/" + NS + "/ublox/fix"
				compass_topic = "/" + NS + "/jmoab_compass"


		print("Subscribing on {:} and {:}".format(gps_topic, compass_topic))

		rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
		rospy.Subscriber(compass_topic, Float32MultiArray, self.compass_callback)

		self.run()
		rospy.spin()

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

	def send_mission_request_int(self, seq):
		#print("send request int {:d}".format(seq))
		self.master.mav.mission_request_int_send(0, 0, seq, 0)

	def send_mission_request(self, seq):
		#print("send request {:d}".format(seq))
		self.master.mav.mission_request_send(0, 0, seq, 0)

	def send_mission_item(self, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z):
		#print("send mission item {:d}".format(seq))
		self.master.mav.mission_item_send(0, 0, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)

	def generate_mission_file(self, mission_file, current_wp, coord_frame, command, param1, param2, param3, param4, lat_list, lon_list, alt_list, autocontinue):
		for i in range(len(lat_list)):
			mission_file.write('{:d}'.format(i))
			mission_file.write("\t")
			mission_file.write("{:d}".format(current_wp[i]))
			mission_file.write("\t")
			mission_file.write("{:d}".format(coord_frame[i]))
			mission_file.write("\t")
			mission_file.write("{:d}".format(command[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param1[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param2[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param3[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param4[i]))
			mission_file.write("\t")
			mission_file.write('{:.16f}'.format(lat_list[i]))
			mission_file.write("\t")
			mission_file.write('{:.16f}'.format(lon_list[i]))
			mission_file.write("\t")
			mission_file.write("{:.2f}".format(alt_list[i]))
			mission_file.write("\t")
			mission_file.write("{:d}".format(autocontinue[i]))
			if i != (len(lat_list)-1):
				mission_file.write("\n")

		mission_file.close()


	def run(self):


		rate = rospy.Rate(50) # 10hz

		while not rospy.is_shutdown():
			
			### HEARTBEAT ###
			if ((time.time() - self.hearbeat_last_send_stamp) > (1.0/self.heartbeat_rate)):
				Type = 10							# https://mavlink.io/en/messages/common.html#MAV_TYPE
				autopilot = 3						# Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. (type:uint8_t, values:MAV_AUTOPILOT)
				base_mode = (64 | 16 | 8 | 4)	# System mode bitmap. (type:uint8_t, values:MAV_MODE_FLAG)
				custom_mode = self.cart_mode		# A bitfield for use for autopilot-specific flags (type:uint32_t)
				system_status = 4					# System status flag. (type:uint8_t, values:MAV_STATE), https://mavlink.io/en/messages/common.html#MAV_STATE
				mavlink_version = 3					# default is 3
				self.master.mav.heartbeat_send(Type, autopilot, base_mode, custom_mode, system_status, mavlink_version)
				self.hearbeat_last_send_stamp = time.time()

			## GPS_RAW_INT ###
			if ((time.time() - self.gpsRawInt_last_send_stamp) > (1.0/self.gps_raw_int_rate)):
				time_usec = int(time.time())	# Unix timestamp
				fix_type = self.gps_fix					# GPS fix type. (type:uint8_t, values:GPS_FIX_TYPE)
				lat = int(self.lat*1E7)			# Latitude (WGS84, EGM96 ellipsoid) [degE7] (type:int32_t)
				lon = int(self.lon*1E7)			# Longitude (WGS84, EGM96 ellipsoid) [degE7] (type:int32_t)
				alt = int(self.alt)			# Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude. [mm] (type:int32_t)
				eph = 65535			# GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
				epv = 65535			# GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
				vel = 65535			# GPS ground speed. If unknown, set to: UINT16_MAX [cm/s] (type:uint16_t)
				cog = 65535			# Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX [cdeg] (type:uint16_t)
				sat_visible = 7	#Number of satellites visible. If unknown, set to 255 (type:uint8_t)
				self.master.mav.gps_raw_int_send(time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, sat_visible)
				self.gpsRawInt_last_send_stamp = time.time()

			### ATTITUDE ###
			if ((time.time() - self.attitude_last_send_stamp) > (1.0/self.attitude_rate)):
				time_boot_ms = int(time.time()/1000.0)					# Timestamp (time since system boot). [ms] (type:uint32_t)
				roll = np.radians(self.roll)												# Roll angle (-pi..+pi) [rad] (type:float)
				pitch = np.radians(self.pitch)											# Pitch angle (-pi..+pi) [rad] (type:float)
				yaw = np.radians(self.hdg)												# Yaw angle (-pi..+pi) [rad] (type:float)
				rollspeed = 0.0											# Roll angular speed [rad/s] (type:float)
				pitchspeed = 0.0 										# Pitch angular speed [rad/s] (type:float)
				yawspeed = 0.0											# Yaw angular speed [rad/s] (type:float)
				self.master.mav.attitude_send(time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
				self.attitude_last_send_stamp = time.time()

			### SYS_STATUS ###
			if ((time.time() - self.sysStatus_last_send_stamp) > (1.0/self.sys_status_rate)):
				onboard_control_sensors_present	= 321969167	# Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. (type:uint32_t, values:MAV_SYS_STATUS_SENSOR)
				onboard_control_sensors_enabled = 271613967	# Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. (type:uint32_t, values:MAV_SYS_STATUS_SENSOR)
				onboard_control_sensors_health = 17858827	# Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy. (type:uint32_t, values:MAV_SYS_STATUS_SENSOR)
				load = 60	# Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000 [d%] (type:uint16_t)
				voltage_battery = 24000	# Battery voltage, UINT16_MAX: Voltage not sent by autopilot [mV] (type:uint16_t)
				current_battery = -1	# Current not sent by autopilot [cA] (type:int16_t)
				battery_remaining = -1 # Battery energy remaining, -1: Battery remaining energy not sent by autopilot [%] (type:int8_t)
				drop_rate_comm = 0 # Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) [c%] (type:uint16_t)
				errors_comm = 0 # Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) (type:uint16_t)
				errors_count1 = 0 # Autopilot-specific errors (type:uint16_t)
				errors_count2 = 0 # Autopilot-specific errors (type:uint16_t)
				errors_count3 = 0 # Autopilot-specific errors (type:uint16_t)
				errors_count4 = 0 # Autopilot-specific errors (type:uint16_t)

				self.master.mav.sys_status_send(onboard_control_sensors_present, \
					onboard_control_sensors_enabled, onboard_control_sensors_health, load, \
					voltage_battery, current_battery, battery_remaining, drop_rate_comm, \
					errors_comm, errors_count1, errors_count2, errors_count3, errors_count4)

				self.sysStatus_last_send_stamp = time.time()

			### SYSTEM_TIME ###
			if ((time.time() - self.sysTime_last_send_stamp) > (1.0/self.sys_time_rate)):
				time_unix_usec = int(time.time())	#time_unix_usec            : Timestamp (UNIX epoch time). [us] (type:uint64_t)
				time_boot_ms = int((time.time() - self.time_boot_ms_start))  #time_boot_ms              : Timestamp (time since system boot). [ms] (type:uint32_t)
				self.master.mav.system_time_send(time_unix_usec, time_boot_ms)
				self.sysTime_last_send_stamp = time.time()


			if ((time.time() - self.print_last_stamp) > (1.0/self.print_rate)):
				print("lat: {:.6f} | lon: {:.6f} | hdg: {:.3f} | fix: {:d}".format(self.lat, self.lon, self.hdg, self.gps_fix))
				self.print_last_stamp = time.time()
			
			rate.sleep()



if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='GPS visualizer on APM Planner2')
	parser.add_argument('--ip',
						help="This is your computer IP for APM Planner")
	parser.add_argument('--mission_dir',
						help="This is where you would like to store mission.txt file")
	parser.add_argument('--id',
						help="ID of rover on APMPlanner")
	parser.add_argument('--ns',
						help="a namespace in front of topics")

	args = parser.parse_args()
	ip = args.ip
	mission_dir = args.mission_dir
	ns = args.ns
	# FCU connection variables

	if ip is None:
		print("Error: please specify GCS computer IP (your PC)")
		quit()

	if mission_dir is None:
		print("Use jmoab-ros/example as the directory")
		mission_dir = os.path.join(os.getcwd(),"mission.txt")

	if args.id is None:
		print("Use id 1 as default")
		_id = 1
	else:
		_id = int(args.id)

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")



	apm_planner = APM(ip, mission_dir, _id, ns)
	# apm_planner = APM(ip)
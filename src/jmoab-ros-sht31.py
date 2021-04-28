#! /usr/bin/env python
import rospy
from smbus2 import SMBus
from sensor_msgs.msg import RelativeHumidity, Temperature
import time

class JMOAB_SHT31:

	def __init__(self):

		rospy.init_node('jmoab_ros_sht31_node', anonymous=True)
		rospy.loginfo("Start JMOAB-ROS-Sensor Temeprature & Humidity node")

		self.bus = SMBus(1)

		self.temp_pub = rospy.Publisher("/jmoab_temp", Temperature, queue_size=10)
		self.temp_msg = Temperature()
		self.humid_pub = rospy.Publisher("/jmoab_humid", RelativeHumidity, queue_size=10)
		self.humid_msg = RelativeHumidity()

		rospy.loginfo("Publishing Temperature data [degree celsius] on /jmoab_temp topic")
		rospy.loginfo("Publishing Relative Humidity [percentage 0.0->1.0] on /jmoab_humid topic")

		## BNO055 address and registers
		self.SENSOR_ADDR = 0x44
		self.DATA_REG = 0x00
		
		self.loop()

		rospy.spin()

	def CRC(self, data):
		crc = 0xff
		for s in data:
			crc ^= s
			for _ in range(8):
				if crc & 0x80:
					crc <<= 1
					crc ^= 0x131
				else:
					crc <<= 1
		return crc

	def loop(self):
		rate = rospy.Rate(20)

		while not rospy.is_shutdown():
			# high repeatability, clock stretching disabled
			self.bus.write_i2c_block_data(0x44, 0x24, [0x00])

			# measurement duration < 16 ms
			time.sleep(0.016)

			# read 6 bytes back
			# Temp MSB, Temp LSB, Temp CRC, Humididty MSB, Humidity LSB, Humidity CRC
			data = self.bus.read_i2c_block_data(self.SENSOR_ADDR, self.DATA_REG, 6)

			if data[2] != self.CRC(data[:2]):
				raise ValueError("temperature CRC mismatch")
			if data[5] != self.CRC(data[3:5]):
				raise ValueError("humidity CRC mismatch")


			temperature = data[0] * 256 + data[1]
			celsius = -45 + (175 * temperature / 65535.0)
			humidity = (data[3] * 256 + data[4]) / 65535.0

			self.temp_msg.header.stamp = rospy.Time.now()
			self.temp_msg.header.frame_id = 'base_link'
			self.temp_msg.temperature = celsius
			self.temp_msg.variance = 0.0

			self.humid_msg.header.stamp = rospy.Time.now()
			self.humid_msg.header.frame_id = 'base_link'
			self.humid_msg.relative_humidity = humidity
			self.humid_msg.variance = 0.0

			self.temp_pub.publish(self.temp_msg)
			self.humid_pub.publish(self.humid_msg)

			rate.sleep()


if __name__ == "__main__":

	jmoab_sht31 = JMOAB_SHT31()
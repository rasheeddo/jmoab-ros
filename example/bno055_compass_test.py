from smbus2 import SMBus
import time
import struct
import numpy as np

def write_pre_calib(pre_calib):
	timeSleep = 0.05
	bus.write_byte_data(IMU_ADDR, ACC_OFF_X_LSB, pre_calib[0])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, ACC_OFF_X_MSB, pre_calib[1])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, ACC_OFF_Y_LSB, pre_calib[2])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, ACC_OFF_Y_MSB, pre_calib[3])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, ACC_OFF_Z_LSB, pre_calib[4])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, ACC_OFF_Z_MSB, pre_calib[5])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_OFF_X_LSB, pre_calib[6])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_OFF_X_MSB, pre_calib[7])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_OFF_Y_LSB, pre_calib[8])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_OFF_Y_MSB, pre_calib[9])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_OFF_Z_LSB, pre_calib[10])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_OFF_Z_MSB, pre_calib[11])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, GYR_OFF_X_LSB, pre_calib[12])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, GYR_OFF_X_MSB, pre_calib[13])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, GYR_OFF_Y_LSB, pre_calib[14])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, GYR_OFF_Y_MSB, pre_calib[15])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, GYR_OFF_Z_LSB, pre_calib[16])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, GYR_OFF_Z_MSB, pre_calib[17])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, ACC_RAD_LSB, pre_calib[18])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, ACC_RAD_MSB, pre_calib[19])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_RAD_LSB, pre_calib[20])
	time.sleep(timeSleep)
	bus.write_byte_data(IMU_ADDR, MAG_RAD_MSB, pre_calib[21])
	time.sleep(timeSleep)

def config_axis_sign(SIGN):
	bus.write_byte_data(IMU_ADDR, AXIS_MAP_SIGN_REG, SIGN)
	time.sleep(0.1)	# 19ms from any mode to config mode

def config_remap(REMAP):
	bus.write_byte_data(IMU_ADDR, AXIS_MAP_CONFIG_REG, REMAP)
	time.sleep(0.1)	# 19ms from any mode to config mode

# Device
IMU_ADDR = 0x28

# REG

EUL_X_LSB = 0x1a
EUL_X_MSB = 0x1b
EUL_Y_LSB = 0x1c
EUL_Y_MSB = 0x1d
EUL_Z_LSB = 0x1e
EUL_Z_MSB = 0x1f

QUA_W_LSB = 0x20
QUA_W_MSB = 0x21
QUA_X_LSB = 0x22
QUA_X_MSB = 0x23
QUA_Y_LSB = 0x24
QUA_Y_MSB = 0x25
QUA_Z_LSB = 0x26
QUA_Z_MSB = 0x27

OPR_MODE = 0x3d

CALIB_STAT = 0x35

AXIS_MAP_CONFIG_REG = 0x41
AXIS_MAP_SIGN_REG = 0x42

ACC_OFF_X_LSB = 0x55
ACC_OFF_X_MSB = 0x56
ACC_OFF_Y_LSB = 0x57
ACC_OFF_Y_MSB = 0x58
ACC_OFF_Z_LSB = 0x59
ACC_OFF_Z_MSB = 0x5a

MAG_OFF_X_LSB = 0x5b
MAG_OFF_X_MSB = 0x5c
MAG_OFF_Y_LSB = 0x5d
MAG_OFF_Y_MSB = 0x5e
MAG_OFF_Z_LSB = 0x5f
MAG_OFF_Z_MSB = 0x60

GYR_OFF_X_LSB = 0x61
GYR_OFF_X_MSB = 0x62
GYR_OFF_Y_LSB = 0x63
GYR_OFF_Y_MSB = 0x64
GYR_OFF_Z_LSB = 0x65
GYR_OFF_Z_MSB = 0x66

ACC_RAD_LSB = 0x67
ACC_RAD_MSB = 0x68

MAG_RAD_LSB = 0x69
MAG_RAD_MSB = 0x6a

# Param
## Mode
CONF_MODE = 0x00
IMU_MODE = 0x08
NDOF_FMC_OFF = 0x0b
NDOF_MODE = 0x0c

## remap axis
REMAP_DEFAULT = 0x24
REMAP_X_Y = 0x21
REMAP_Y_Z = 0x18
REMAP_Z_X = 0x06
REMAP_X_Y_Z_TYPE0 = 0x12
REMAP_X_Y_Z_TYPE1 = 0x09

## axis sign
SIGN_DEFAULT = 0x00	# if heatsink is down
Z_REV = 0x01			# if heatsink is up
XZ_REV = 0x04		#
YZ_REV = 0x03		# heatsink is up, usb is forward



calib_stat = 0x00
# pre_calib = [211, 255, 239, 255, 230, 255, 68, 1, 252, 0, 166, 1, 253, 255, 255, 255, 255, 255, 232, 3, 155, 0]
# pre_calib = [215, 255, 250, 255, 217, 255, 42, 1, 52, 1, 223, 1, 253, 255, 255, 255, 255, 255, 232, 3, 142, 0]
# pre_calib = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 224, 1]
# pre_calib = [217, 255, 6, 0, 245, 255, 84, 1, 248, 0, 156, 0, 255, 255, 255, 255, 255, 255, 232, 3, 232, 1]
# pre_calib = [249, 255, 251, 255, 0, 0, 136, 0, 140, 254, 24, 255, 254, 255, 255, 255, 0, 0, 232, 3, 108, 3]

pre_calib = []
with open("calibration_offset.txt", "r") as f:
	for line in f:
		pre_calib.append(int(line.strip()))

print("Read calibration_offset.txt file")
print(pre_calib)


bus = SMBus(1)

bus.write_byte_data(IMU_ADDR, OPR_MODE, CONF_MODE)
time.sleep(0.1)

config_axis_sign(YZ_REV)

write_pre_calib(pre_calib)

bus.write_byte_data(IMU_ADDR, OPR_MODE, NDOF_FMC_OFF)
time.sleep(0.1)



offset_data = bus.read_i2c_block_data(IMU_ADDR, ACC_OFF_X_LSB, 22)
print("offset_data from IMU register", offset_data)
time.sleep(0.1)

## some time the sensor starts from 0.0 even after loaded pre_calib
## so we just quit and start again.
## after few reset, it will give the correct heading
while True:
	startTime = time.time()
	raw = bus.read_i2c_block_data(IMU_ADDR, EUL_X_LSB, 6)
	hdg,roll,pitch = struct.unpack('<hhh', bytearray(raw))

	hdg = hdg/16.0
	roll = roll/16.0
	pitch = pitch/16.0

	if (hdg == 0.0) or (hdg == 360.0):
		print("heading is 0.0, try running the code again")
		quit()
	else:
		break


lastTimePrint = time.time()

while True:
	startTime = time.time()
	raw = bus.read_i2c_block_data(IMU_ADDR, EUL_X_LSB, 6)
	hdg,roll,pitch = struct.unpack('<hhh', bytearray(raw))

	hdg = hdg/16.0
	roll = roll/16.0
	pitch = pitch/16.0

	## print only 1 message per sec
	if (time.time() - lastTimePrint) > 1.0:
		print("hdg: {:.3f} | roll: {:.3f} | pitch: {:.3f} ".format(hdg, roll, pitch))
		lastTimePrint = time.time()


	# raw = bus.read_i2c_block_data(IMU_ADDR, QUA_W_LSB, 8)
	# qw,qx,qy,qz = struct.unpack('<hhhh', bytearray(raw))

	# qw = qw/16384.0
	# qx = qx/16384.0
	# qy = qy/16384.0
	# qz = qz/16384.0

	# r11 = qw**2 + qx**2 - qy**2 - qz**2 #1 - 2*qy**2 - 2*qz**2
	# r12 = 2*qx*qy - 2*qz*qw
	# r13 = 2*qx*qz + 2*qy*qw
	# r21 = 2*qx*qy + 2*qz*qw
	# r22 = qw**2 - qx**2 + qy**2 - qz**2	#1 - 2*qx**2 - 2*qz**2
	# r23 = 2*qy*qz - 2*qx*qw
	# r31 = 2*qx*qz - 2*qy*qw
	# r32 = 2*qy*qz + 2*qx*qw
	# r33 = qw**2 - qx**2 - qy**2 + qz**2	#1 - 2*qx**2 - 2*qy**2
	# rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

	# ## default orientation (heatsink is pointing down)
	# rot_matrix = np.array([
	# 					[ 1, 0, 0],
	# 					[ 0, 1, 0],
	# 					[ 0, 0, 1]])

	# rot = np.dot(rot,rot_matrix)

	# yaw2 = np.arctan2(rot[1,0],rot[0,0])
	# pitch = np.arcsin(-rot[2,0])
	# roll = np.arctan2(rot[2,1],rot[2,2])

	# print("yaw2 %.3f| roll %.3f | pitch %.3f" %(np.degrees(yaw2),np.degrees(pitch),np.degrees(roll)))


	period = time.time() - startTime 
	# print("period", period)
	if period < 0.007:
		sleepMore = 0.007 - period
		time.sleep(sleepMore)


from icm20948 import ICM20948
import time
import numpy as np
import os

## magnetometer is in uT (micro Tesla)
## gyroscope is in dps (degree per second)
## accelerometer is in G (9.81 m/s^2)

imu = ICM20948(i2c_addr=0x69)

# imu.bank(2)
# imu.set_gyro_sample_rate(100)
# imu.set_gyro_low_pass(enabled=True, mode=5)
# imu.set_gyro_full_scale(250)
# imu.set_accelerometer_sample_rate(125)
# imu.set_accelerometer_low_pass(enabled=True, mode=5)
# imu.set_accelerometer_full_scale(2)

wait_time = 10.0
sample_data = 200

calib_seq = ["x pointing UP", "x pointing DOWN",
			"y pointing UP", "y pointing DOWN",
			"z pointing UP", "z pointing DOWN"
			]

# accel_mean_list = [x_pos_mean, x_neg_mean, y_pos_mean, y_neg_mean, z_pos_mean, z_neg_mean]
accel_mean_list = np.array([])

# scale_factor_list = [ax_scale_factor, ay_scale_factor, az_scale_factor]
scale_factor_list = np.array([])

# bias_list = [ax_bias, ay_bias, az_bias]
bias_list = np.array([])

# mx, my, mz = imu.read_magnetometer_data()
# ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

# print("ax {:05.2f} ay {:05.2f} az {:05.2f} | gx {:05.2f} gy {:05.2f} gz {:05.2f} | mx {:05.2f} my {:05.2f} mz {:05.2f}".format(\
# 	ax, ay, az, gx, gy, gz, mx, my, mz))


print("________ Accelerometer calibration __________")

for i, seq in enumerate(calib_seq):

	print("Placing {:}... then press any key".format(seq))
	raw_input()
	print("Start getting data...Don't move IMU!")
	buf = np.array([])

	for ii in range(sample_data):

		ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
		print("ax {:05.2f} ay {:05.2f} az {:05.2f} | gx {:05.2f} gy {:05.2f} gz {:05.2f}".format(\
			ax, ay, az, gx, gy, gz))
		## X axis
		if (i == 0) or (i == 1):
			sensor_raw = ax
		## Y axis
		elif (i == 2) or (i == 3):
			sensor_raw = ay
		## Z axis
		elif (i == 4) or (i == 5):
			sensor_raw = az

		
		buf = np.append(buf, sensor_raw)
		time.sleep(0.02)

	print(buf)
	mean = np.mean(buf)
	print("Mean value {:.4f}, this should be close to -1 or 1".format(mean))
	accel_mean_list = np.append(accel_mean_list, mean)

ax_scale_factor = 2.0/(accel_mean_list[0] - accel_mean_list[1])
ax_bias = -(accel_mean_list[0] + accel_mean_list[1])/(accel_mean_list[0] - accel_mean_list[1])

ay_scale_factor = 2.0/(accel_mean_list[2] - accel_mean_list[3])
ay_bias = -(accel_mean_list[2] + accel_mean_list[3])/(accel_mean_list[2] - accel_mean_list[3])

az_scale_factor = 2.0/(accel_mean_list[4] - accel_mean_list[5])
az_bias = -(accel_mean_list[4] + accel_mean_list[5])/(accel_mean_list[4] - accel_mean_list[5])

print("ax_scale_factor: {:.4f} | ax_bias: {:.4f}".format(ax_scale_factor, ax_bias))
print("ay_scale_factor: {:.4f} | ay_bias: {:.4f}".format(ay_scale_factor, ay_bias))
print("az_scale_factor: {:.4f} | az_bias: {:.4f}".format(az_scale_factor, az_bias))

#######################################################################################################
gx_buf = np.array([])
gy_buf = np.array([])
gz_buf = np.array([])


print("________ Gyroscope calibration __________")

print("Placing the IMU stationary on flat surface, then press any key")
raw_input()
print("Start getting data...Don't move IMU!")
for j in range(sample_data):

	ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

	gx_buf = np.append(gx_buf, gx)
	gy_buf = np.append(gy_buf, gy)
	gz_buf = np.append(gz_buf, gz)

	time.sleep(0.02)

gx_bias = np.mean(gx_buf)
gy_bias = np.mean(gy_buf)
gz_bias = np.mean(gz_buf)

print("gx_bias: {:.4f}".format(gx_bias))
print("gy_bias: {:.4f}".format(gy_bias))
print("gz_bias: {:.4f}".format(gz_bias))

#######################################################################################################
print("________ Magnetometer calibration __________")

mx_list = []
my_list = []
mz_list = []

print("________ Magnetometer plot __________")
print("If ready then press any key and start to move IMU around in the air")
raw_input()
print("Start getting data..")
for i in range(2000):

	mx, my, mz = imu.read_magnetometer_data()

	mx_list.append(mx)
	my_list.append(my)
	mz_list.append(mz)

	print("i: {:d} mx {:05.2f} my {:05.2f} mz {:05.2f}".format(\
		i, mx, my, mz))

	time.sleep(0.001)

## Hard Iron corrected
mx_bias = (np.max(mx_list) + np.min(mx_list))/2
my_bias = (np.max(my_list) + np.min(my_list))/2
mz_bias = (np.max(mz_list) + np.min(mz_list))/2

mx_cal_list = mx_list - mx_bias
my_cal_list = my_list - my_bias
mz_cal_list = mz_list - mz_bias

## Soft Iron corrected
avg_delta_x = (np.max(mx_cal_list) - np.min(mx_cal_list))/2
avg_delta_y = (np.max(my_cal_list) - np.min(my_cal_list))/2
avg_delta_z = (np.max(mz_cal_list) - np.min(mz_cal_list))/2

avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z)/3

mx_scale_factor = avg_delta/avg_delta_x
my_scale_factor = avg_delta/avg_delta_y
mz_scale_factor = avg_delta/avg_delta_z

mx_full_cal_list = mx_cal_list*mx_scale_factor
my_full_cal_list = my_cal_list*my_scale_factor
mz_full_cal_list = mz_cal_list*mz_scale_factor

print("mag_offset_x: {:.4f} | mag_offset_y: {:.4f} | mag_offset_z: {:.4f}".format(mx_bias, my_bias, mz_bias))
print("mag_scale_x: {:.4f} | mag_scale_y: {:.4f} | mag_scale_z: {:.4f}".format(mx_scale_factor, my_scale_factor, mz_scale_factor))


#######################################################################################################
file = open("icm20948_offset.txt", "w+")
file.write(str(ax_scale_factor))
file.write("\n")
file.write(str(ax_bias))
file.write("\n")
file.write(str(ay_scale_factor))
file.write("\n")
file.write(str(ay_bias))
file.write("\n")
file.write(str(az_scale_factor))
file.write("\n")
file.write(str(az_bias))
file.write("\n")
file.write(str(gx_bias))
file.write("\n")
file.write(str(gy_bias))
file.write("\n")
file.write(str(gz_bias))
file.write("\n")
file.write(str(mx_scale_factor))
file.write("\n")
file.write(str(mx_bias))
file.write("\n")
file.write(str(my_scale_factor))
file.write("\n")
file.write(str(my_bias))
file.write("\n")
file.write(str(mz_scale_factor))
file.write("\n")
file.write(str(mz_bias))



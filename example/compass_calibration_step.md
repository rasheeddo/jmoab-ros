# BNO055 Compass calibration step

- make sure the Jetson with JMOAB is not lock with the vehicle (in case of medium and huge cart)
- open the file `example/bno055_compass_calibrate.py`, and make sure the sensor's orientation is placed correctly with `config_axis_sign()` and `config_remap()` functions inside.
- then run `example/bno055_compass_calibrate.py` it will print all of sensors status, we need to make all sensors status equal to 3
	- gyro calibration: keep it steady in place will make it success
	- acc calibration: place it in some angles like 45,90,180,-45,-90 wil make it success
	- mag calibration: moving the whole board in the air, draw "8" in the air would make it success
	- once all succeed, it will print the sensor offset, make sure it's not all 0
	- if it's showing 0, then try run the code again, it will show the correct offset immediately without doing from first step again.
- we can make sure our compass is working by running `example/bno055_compass_test.py`, comfirm the heading angle with other compass source (mobile phone)

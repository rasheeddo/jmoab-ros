# BNO055 Compass calibration step

- make sure the Jetson with JMOAB is not lock with the vehicle (in case of medium and huge cart)
- run `example/bno055_compass_calibrate.py` it will print all of sensors status, we need to make all sensors status equal to 3
-- gyro calibration: keep it steady in place will make it success
-- acc calibration: place it in some angles like 45,90,180,-45,-90 wil make it success
-- mag calibration: moving the whole board in the air, draw "8" in the air would make it success
-- once all succeed, it will print the sensor offset, make sure it's not all 0
- we can make sure our compass is working by running `example/bno055_compass_test.py`, comfirm the heading angle with other compass source (mobile phone)
- create a blank txt file on example directory as `heading_offset.txt` and put 0.0 on it, this is for when we mount the Jetson and JMOAB on cart, there would be some heading offset with the true cart's heading
- run these
-- `roslaunch ublox_gps ublox_device.launch node_name:=ublox param_file_name:=zed_f9p` make sure we have this package with proper config file
-- ```
	cd /home/nvidia/RTKLIB/app/consapp/str2str/gcc
	./str2str -in ntrip://rtk2go.com:2101/KAIT-RTCM3 -out serial://ttyACM0:115200 -b 1
	```
	this is to run RTK and make GPS more accurate, make sure to have RTKLIB install and GPS USB port connect as ttyACM0
-- `rosrun jmoab-ros jmoab-ros-compass.py` to start using compass with sensor offset, it will load heading_offset as 0.0 from the file we created above.
-- `rosrun jmoab-ros jmoab-ros-atcart.py` to get access of sbus_rc_ch topic
-- `example/apm_planner_visualize.py --ip <your_pc_ip>` to visualize the GPS point on map, make sure we can have correct heading roughly
-- on example directory, `python get_offset_heading.py`, you will need to wait for few second to let the code get data, then move the bot as straight line in manual mode for 4-5 meters, the trig the Futaba ch9 to high then it will calculate the robot's heading_offset and save to `heading_offset.txt`

- once you got the correct heading_offset of the bot, you have to rerun the `jmoab-ros-compass.py` again
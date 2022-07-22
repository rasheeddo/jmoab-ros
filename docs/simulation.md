## JMOAB with Gazebo model and SITL

To run JMOAB ROS as SITL (without the actual hardware), at least we should have a gamepad for a robot controller in order to change mode, steering/throttle, and some switches. I am using Logicool F310, and the axes and buttons are assinged as below

![](images/joy.png)

### Dependencies

- Make sure you have `joy` package installed, `sudo apt-get install ros-$ROS_DISTRO-joy`.

- You will need to make sure, that the Gazebo's URDF file has `libgazebo_ros_imu_sensor` and `libhector_gazebo_ros_gps` plugins, to have the `ublox/fix` and `imu` topics publishing. So this URDF file depends on which model are using, and it's not in this package.

### Run

	rosrun joy joy_node

	rosrun jmoab-ros atcart_basic_sim.py

Start atcart simulation node, this will start an `atcart` node simulation which going to pubish `cart_mode` as current mode, and to subscribe `cart_mode_cmd` to get command mode, and `wheels_cmd` for sbus values of steering and throttle. So we can have the same manner of topics similar to the real cart. The robot mode in gazebo is using `cmd_vel` topic for drive the wheel, so this node will publish the `cmd_vel` according to the input of `sbus_cmd` topic or `joy` node from user manual control.

Once you change the mode by pressing X button on gamepad, the rover will be in manual mode, so you can drive it by throttle and steering analog stick as shown on image above. And once you pressed A button on gamepad, it will be in auto mode, and will listen on `sbus_cmd` topic for an autonomous drive program. B button is for hold mode, so it will stop moving. 

	rosrun jmoab-ros jmoab-ros-compass-simulation.py

Start a compass simulation node, this will be useful for outdoor GPS navigation, so you need to make sure there is `imu` topic publishing before. This node will publish `jmoab_compass` topic which is `[roll, pitch, heading]` std_msgs/Float32MultiArray the same as original `jmoab_compass` message. And also you can press Y button to do the `heading calibration` the same mannaer as `jmoab-compass.py` node as explained [at the end here](https://github.com/rasheeddo/jmoab-ros/blob/master/example/compass_calibration_step.md). Please make sure you have `ublox/fix` topic publishing.

	rosrun jmoab-ros jmoab-ros-adc-simulation.py

This is going to start a battery measurement simulation node, it's specified to use with 6S LiPo battery, so with fully charged it's 25.2V, and it could stay around 6 hours for a small cart. So first element of `/jmoab_adc` is a battery remaining voltage. We could try to simulate other Analog-To-Digital signal later on.


Or for a better convenience, we could run all of required nodes in a launch file as a command below

	roslaunch jmoab-ros jmoab-ros-simulation.launch

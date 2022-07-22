## JMOAB with ZLAC8015D Driver

The ZLAC8015D is dual brusless motor driver for 8 inch wheel. It could be used in both position and velocity control modes.

In order to use `jmoab-ros-zlac8015d.py` node, we have to install [this python API](https://github.com/rasheeddo/ZLAC8015D_python_API). 

Assuming that you cloned the above API to `/home/<Your-Username>/ZLAC8015D_python_API/` directory. You will need to link the `ZLAC8015D.py` class to `jmoab-ros/src` directory. Please run the command below, and replace `<Your-Username>` to correct username.

	ln -s /home/<Your-Username>/ZLAC8015D_python_API/ZLAC8015D.py /home/<Your-Username>/catkin_ws/src/jmoab-ros/src/ZLAC8015D.py

This will create the symlink of `ZLAC8015D.py` into `jmoab-ros/src`. So the `jmoab-ros-zlac8015d.py` can see the file.

### Topics

This node is publishing data as following

- `/odom`, we get the data from wheel's encoder as RPM in velocity control and wheel's travelled in position control, those data are used to calculate the robot odometry data. You will need to change `self.L` according to your wheel's base width.

- `/wheels_rpm`, the speed in RPM of each wheel as [left, right]

- `/sbus_rc_ch` and `/atcart_mode`, similar to ATCart.

This node is subscribing on,

- `/sbus_cmd` and `/atcart_mode_cmd`, similar to ATCart.

- `/zlac8015d/mode_cmd`, to change the ZLAC8015D mode, 1 for position control, 3 for velocity control

- `/zlac8015d/pos/deg_cmd`, to send command as angle degree in position control, e.g. [90,70] 90deg of left wheel, 70deg of right wheel.

- `/zlac8015d/pos/dist_cmd`, to send command as desired travelling distance, this would be correct if you are using 8 inch motors as default, e.g. [1.0, 1.0] for one meter travelling distance of each wheel.  

Please check on [this video](https://youtu.be/3gUNfJ94oac) for a demonstration.

### Odometry

Raw odometry of ZLAC8015D is good enough in some application, but for an indoor navigation which the robot would move around the room, the `/odom` topic can suffer from tire's slip and other factor which could make the odometry data got offset.

To get a better odometry, I would suggest to use [robot_pose_ekf](http://wiki.ros.org/robot_pose_ekf) to fuse IMU data with pure odometry. So please clone this package to your catkin workspace.

I changed [this](https://github.com/ros-planning/robot_pose_ekf/blob/fd6cef32b447e8b344a1111373e515aa2f8bfc50/robot_pose_ekf.launch#L5) `base_footprint` to `base_link`, and disable [this](https://github.com/ros-planning/robot_pose_ekf/blob/fd6cef32b447e8b344a1111373e515aa2f8bfc50/robot_pose_ekf.launch#L10) `vo_used` to false.

You can run the launch file below to start all necessary nodes,

	roslaunch jmoab-ros jmoab-ros-ekf-odom-zlac8015d.launch

This will start, 

- `jmoab-ros-zlac8015d.py`, to get pure wheel's odometry

- `jmoab-ros-imu.py`, to get jmoab_imu 

- `ekf_odom_generate.py`, a helper script to convert `/robot_pose_ekf/odom_combined` which doesn't have TF to `/ekf_odom` which has a proper TF.

- `robot_pose_ekf.launch`, ekf node to fuse data.

- and some static transform publisher.

Please check on [this video](https://youtu.be/XNgGo2GZ2M8) for a comparison of with/without `robot_pose_ekf`.
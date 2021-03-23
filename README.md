# JMOAB-ROS

This is a ros package for AT_JMOAB01 development shield on Jetson Nano/Xavier and Raspberry Pi for motor and I/O control.

![](images/nano.jpg)

## Install

Install smbus2 python module for i2c communication 
- `pip install smbus2`

Clone this repo to src directory of catkin workspace
- `cd ~/catkin_ws/src`
- `git clone https://github.com/rasheeddo/jmoab-ros.git`

Make sure jmoab-ros script is executable
- `cd ~/catkin_ws/src/jmoab-ros/src`
- `chmod +x jmoab-ros.py`

Checking that your computer (Jetson or Pi) could detect this device
- `i2cdetect -y -r 1` then you should see a device address of 0x71

Build a package
- `cd ~/catkin_ws`
- `catkin_make`

Source environment
- `source ~/.bashrc`
- `source ~/catkin/devel/setup.bash`


## Run

Run the jmoab-ros node
- `rosrun jmoab-ros jmoab-ros.py`, 

List out topics
- `rostopic list`, you would see there are two topics got generated `/sbus_rc_ch` for publishing SBUS RC channel, and `/sbus_cmd` for subscribing a steering and throttle commands from your autopilot code.

Check more detail on example scripts
- `rosrun jmoab-ros sbus_ch_listener.py` for test reading sbus channel on a script
- `rosrun jmoab-ros sbus_cmd_sender.py` for test writing a command steering and throttle from a script


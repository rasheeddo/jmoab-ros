## Wheel's Odometer

![](../images/hall_wheels.png)

### Hardware

You will need the parts below

- 4pcs of Hall effect IC SK8552G-G03-K 
- 16pcs of 6x3 magnets
- 2pcs of sensors holder
- 2pcs of magnets ring
- 1pc of Arduino nano
- 1pc of USB-B mini type

![](../images/actual_sensor.jpeg)

### Software

The source code to read sensor and calculate RPM is written in Arduino C++, you can check the file on [here](../arduino/hall_odom_2wheels_PCINT/hall_odom_2wheels_PCINT.ino).

And the source code to use this RPMs on ros node is `jmoab-ros-wheels-rpm.py`


### Install

- attaching a magnets ring on the wheel hub motor, please tight screws of both side evenly.
- attaching a sensors holder on the cart's frame. On the left wheel side, the sensor's cable should be makred as 8 and 9, and on the right side, the sensor cable should be marked as 10 and 12. Those numbers are corresonding to the Arduino pins.
- adjusting the sensor to close to the magnets ring around 5mm

![](../images/adjusting_sensor.jpeg)

- plug all the sensor cables to Arduino nano, and plug USB port to computer.
- make sure the computer could detect USB device by using `ls /dev/tty*`, there would be `/dev/ttyUSB0` existing on the list.
- add permission to access USB device, `sudo usermod -a -G dialout nvidia`, then reboot. This is only one time setup.
- run roscore on one of terminal, then run this node `rosrun jmoab-ros jmoab-ros-wheels-rpm.py /dev/ttyUSB0`, make sure your USB device is correct on the argument.
- if there is no error, you should see `/wheels_rpm` topic. The first array element is rpm on the left wheel, and second element is rpm on the right wheel.


***NOTE***
The minimum RPM that this sensor can read is +/-5 rpm, lower than this value it will give 0 rpm.

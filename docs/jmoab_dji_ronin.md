## JMOAB with DJI Ronin-SC control

DJI Ronin-SC handheld camera stabilizer is DSLR camera gimbal. It could be remotely operated by RC transmitter with SBUS signal. For more detail how, please check on this [video](https://www.youtube.com/watch?v=fCnYqv7fR_c&ab_channel=Mad%27sTech). 

In order to let the Jetson control the gimbal, we need to use a special JMOAB firmware (v07e_20210805_AT_JMOAB01&03_SBUSOUT.hex) which able to control ATCart wheels and also SBUS output on SBUS3 port. It could be found [here](./firmwares/)

We need to use `jmoab-ros-atcart-gimbal.py` to control the cart and gimbal.

There is still `/sbus_cmd` topic which going to receive sbus steering and throttle commands similar to normal `jmoab-ros-atcart.py`, and we have new topics as `/sbus_gimbal_cmd` which is the speed of pan and tilt of gimbal. For example, sending `[1680, 1024]` on this will make the gimbal panning in full speed, or `[1024, 1680]` will make the gimbal tilting in full speed. And there is `/gimbal_recenter` which is a boolean topic to recenter the gimbal back to home position. Send `True` to recenter the gimbal, and only needs one time (not continuous).


To publish custome topics for debugging, please check on this [rqt_ez_publisher package](http://wiki.ros.org/rqt_ez_publisher).
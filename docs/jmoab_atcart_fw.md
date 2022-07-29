## ATCart ESC FW

#### Remarks 

`jmoab-ros-atcart.py` works only with the firmwares of

- v07_20210614_AT_JMOAB01&03.hex

- v07a_20210728_AT_JMOAB01&03.hex

- v07e_20210805_AT_JMOAB01&03_SBUSOUT.hex

For a better stability in autonomous navigation, I highly recommend to use the firmwares of 

- 20220117_AT_JMOAB01_fw_v07.2b_skidsteer.hex 

or 

- 20220117_AT_JMOAB05_fw_v07.2b_skidsteer.hex

Please check all of the firmwares [here](../firmwares/)

This `_skidsteer` version has more evenly controlable in both wheels, because it's non-mixing mode, so we could control each wheel individually. So during steering or skidding, the cart has more precise control.

Please use `jmoab-ros-atcart-diff-drive.py` for both firmwares above. This script has done the mixing mode inside, so we could still publish the same topic as `/sbus_cmd` for [steering, throttle] but the result is much better than default mixing by PSoc controller.

#### Remarks 2

With the new batch of ATCart's ESC with model name of "MN1 WSDC/3+7F&G-X"

![](../images/new_batch_esc_SN.jpeg)

![](../images/new_batch_joystick.jpeg)

The supplier has changed some handshake package and data ranges, please check the detail on [this repo](https://github.com/rasheeddo/BrushlessDriveWheels/tree/new-batch-esc#hack-new-batch-esc). So we need to upgrade the new firmware too, to be able to use with this new batch ESC. The firmware of new ESC is as following

- 20220127_AT_JMOAB01_fw_v08.2_skidsteer_newESC_steer_REV.hex

- 20220127_AT_JMOAB04_fw_v08.2_skidsteer_newESC_steer_REV.hex

- 20220127_AT_JMOAB05_fw_v08.2_skidsteer_newESC_steer_REV.hex

Those three firmware has the same function, but only the hardware of the JMOAB version is different.
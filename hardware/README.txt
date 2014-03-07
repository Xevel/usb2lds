USB2LDS hardware
================







USB2LDS versions:

v2.0 - 2014/01/12
- replaced FTDI with Atmega32u2
- added mosfet to allow PWM control of the motor
- added debug and hacking pads (/RST, /HWB, and the whole SPI bus)

v1.1 - 2014/01/11
- corrected LDS_TX/LDS_RX inversion on the connector
- The Lidar needs more power, use 3.3V regulator power instead of 3V3OUT from the FT232RL

V1.0 - 2014/01/09
- inital release
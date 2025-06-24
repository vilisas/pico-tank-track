Cheap robot track controlled by Raspberry Pi Pico W
and pca9685 pico motor driver

Driver from https://github.com/adafruit/micropython-adafruit-pca9685

Track: 
https://www.aliexpress.com/item/1005007809968339.html

4-Channel DC Motor Driver Controller HAT:
https://www.aliexpress.com/item/1005007930673173.html

Raspberry Pi Pico W

* Wiring *

Left track:
A1      Motor +
A2      Motor -

Right track:
B1:     Motor +
B2:     Motor -

ELRS:
GPIO1: TX (connect to receiver's RX)
GPIO2: RX (connect to receiver's TX)
Protocol CRSF
Baudrate: 420000


upload pca9685.py to rpi board, then execute motor_driver.py

Channel configuration:

CH1: Yaw
CH3: Throttle
CH5: Arm / Disarm (move sticks to center to arm)


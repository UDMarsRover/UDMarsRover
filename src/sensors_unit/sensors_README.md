# Hello!
This is the documentation on the sensors code.

The gps_mph_deg_accel_ros2.ino is the working code that should be put on the Ardunio Due
### Note it will (most likely) not work on the Mega or Uno.

The other file in the test_code folder were examples I used to test, but should not be used in the rover final product.

When the gps_mph_deg_accel_ros2.ino file is loaded onto the Arduino, you will need to hit the reset button on the Arudino during compilation, or the aruino may fail to connect to the computer.
(This is becuase the Arduino is in communication with the Raspberry Pi and ignores the programming computer.)

# Wireing
Computer plugs into programming port
RPi plugs into native port

GPS Rx and Tx should be plugged in to Rx and Tx 1 on the Arduino.

If it doesn't work connecting to the pi, try a differnt USB port.

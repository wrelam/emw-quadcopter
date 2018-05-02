This project demonstrates transmitting and receiving the following quadcopter
controls: roll, pitch, yaw, and thrust. Trims for roll, pitch, and yaw are also
supported via a web page. A BNO055 IMU is attached and samples orientation
information at a maximum of 100Hz while displaying it over the Serial connection
every 5 seconds.

The overall design uses an XBox 360 controller connected to a RaspberryPi (RPi)
which sends control information to an ESP8266 board attached to an Arduino. The
Arduino can directly copy the command into its buffer for later execution. A
server is also running through the ESP8266 so that status information can be
reported wirelessly over HTTP.

This uses a mock setup to verify that the overall design works, where
directional commands turn on certain LEDs and thrust commands spin up/down a
small motor with fan attached. This test design should map very well to the
actual quadcopter hardware.

We're using the Raspbian distribution of Linux with the following packages:

    * xboxdrv - XBox 360 controller driver


This project demonstrates transmitting and receiving the following quadcopter
controls: roll, pitch, yaw, and thrust. Trims for roll, pitch, and yaw are also
supported.

The overall design uses an XBox 360 controller connected to a RaspberryPi (RPi)
which sends control information to another RaspberryPi over WiFi. The RPi with
the controller attached is the ground station while the RPi receiving controls
will be on the quadcopter itself. The receiving RPi sends these commands to an
Arduino which can interface with the ESCs or a PID controller board.

The ground station uses the xboxdrv driver to capture events from the controller
which are delivered to a Python script. This script performs a translation
between the controller event and a specific command. For example, it will map
the movement of a joystick forward to creating an "increase thrust" command.
Once the command is created, it is sent over a UDP socket to the RPi on the
quadcopter. The quadcopter RPi is connected to an Arduino via i2c which provides
an easy communications path between the two pieces of hardware. Once a command
is received by the RPi, it dispatches it to the Arduino which carries out the
command, such as providing more power to a motor when an "increase thrust"
command is received.

This uses a mock setup to verify that the overall design works, where
directional commands turn on certain LEDs and thrust commands spin up/down a
small motor with fan attached. This test design should map very well to the
actual quadcopter hardware.

We're using the Raspbian distribution of Linux with the following packages:

    * xboxdrv - XBox 360 controller driver
    * i2c-tools - i2c communications software


#!/usr/bin/env python
################################################################################
#
# client.py
#
# Captures events from the controller and sends them to the server.
#
################################################################################
import socket
import struct
import sys
import time

import XboxController

from common import *

# Create the socket for sending commands, using argv[1] if provided
servHost = 'localhost'
if 1 < len(sys.argv):
    servHost = sys.argv[1]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
servAddr = (servHost, COMMAND_PORT)
sock.connect(servAddr)

# Send a command to the server
def sendCmd(cmd):
    print("Sending cmd: " + strCmd(cmd))
    packer = struct.Struct('B')
    data = packer.pack(cmd)
    sock.sendall(data)

# Generic callback, useful for debugging XBox controller
def genCb(controlId, value):
    print(" Control Id = {}, Value = {}".format(controlId, value))

# Start button callback
def startCb(value):
    if value == 1:
        sendCmd(CMD_START)

# Back button callback
def backCb(value):
    if value == 1:
        sendCmd(CMD_STOP)

# Left joystick X-axis callback
# Positive rolling motion lowers the right wing (tilt right)
# joystick right generates positive rolling motion
def leftJoyXCb(value):
    if value < 0:
        sendCmd(CMD_ROLL_NEG)
    elif value > 0:
        sendCmd(CMD_ROLL_POS)

# Left joystick Y-axis callback
# Positive pitching motion moves the nose up (tilt up)
# joystick down generates positive pitching motion
def leftJoyYCb(value):
    if value < 0:
        sendCmd(CMD_PITCH_NEG)
    elif value > 0:
        sendCmd(CMD_PITCH_POS)

# Right joystick X-axis callback
# Positive yawing motion moves the nose to the right (clockwise spin)
# joystick right generates positive yawing motion
def rightJoyXCb(value):
    if value < 0:
        sendCmd(CMD_YAW_NEG)
    elif value > 0:
        sendCmd(CMD_YAW_POS)

# Right joystick Y-axis callback
# Positive thrust moves the aircraft up
# jostick up generates positive thrust
def rightJoyYCb(value):
    if value < 0:
        sendCmd(CMD_THRUST_POS)
    elif value > 0:
        sendCmd(CMD_THRUST_NEG)

# Create a controller object and assign callbacks
xboxCont = XboxController.XboxController(controllerCallBack = None,
                                         joystickNo = 0,
                                         deadzone = 0.25,
                                         scale = 1,
                                         invertYAxis = False)

xboxCont.setupControlCallback(xboxCont.XboxControls.START, startCb)
xboxCont.setupControlCallback(xboxCont.XboxControls.BACK, backCb)
xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBX, leftJoyXCb)
xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBY, leftJoyYCb)
xboxCont.setupControlCallback(xboxCont.XboxControls.RTHUMBX, rightJoyXCb)
xboxCont.setupControlCallback(xboxCont.XboxControls.RTHUMBY, rightJoyYCb)

try:
    xboxCont.start()
    print("Controller running")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print "User cancelled"
except:
    print "Unexpected error: ", sys.exc_info()[0]
    raise
finally:
    xboxCont.stop()
    sock.close()


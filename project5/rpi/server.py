#!/usr/bin/env python
################################################################################
#
# server.py
#
# Receives quadcopter commands and sends them over the i2c bus to an Arduino.
#
################################################################################
import smbus
import socket
import struct
import sys

from common import *

# Establish i2c bus connection
bus = smbus.SMBus(I2C_DEVICE_BUS)

# Create socket for receiving commands
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
servAddr = ('0.0.0.0', COMMAND_PORT)
sock.bind(servAddr)

# Data (un)packer, not necessarily required for single bytes
unpacker = struct.Struct('B')

while True:
    # Wait for data
    try:
        data = sock.recv(unpacker.size)
    except KeyboardInterrupt:
        print("User cancelled")
        sock.close()
        sys.exit(0)
    except:
        print("Error: ", sys.exc_info()[0])
        continue

    # Process received data
    unpacked = unpacker.unpack(data)
    cmd = unpacked[0]
    print("Received cmd: " + strCmd(cmd))

    # Send data over the i2c bus
    try:
        bus.write_byte(I2C_DEVICE_ADDR, cmd)
    except:
        # These aren't necessarily bad, the Arduino may be busy and not
        # accepting commands.
        e = sys.exc_info()[0]
        print("i2c write error: ")
        print e


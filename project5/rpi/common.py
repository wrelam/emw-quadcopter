#!/usr/bin/env python
################################################################################
#
# common.py
#
# Common definitions for Python scripts
################################################################################

# i2c information
I2C_DEVICE_BUS  = 1
I2C_DEVICE_ADDR = 0x10

# Network information
COMMAND_PORT    = 5000

# Quadcopter commands
# Generically these follow the right-hand rule: CCW positive, CW negative
#   Pitch:  thumb forward
#   Roll:   thumb right
#   Yaw:    thumb up
CMD_START       = 0x41  # Spin motors
CMD_STOP        = 0x42  # Turn off motors
CMD_THRUST_POS  = 0x43  # Up
CMD_THRUST_NEG  = 0x44  # Down
CMD_PITCH_POS   = 0x45  # Backward
CMD_PITCH_NEG   = 0x46  # Forward
CMD_ROLL_POS    = 0x47  # Right
CMD_ROLL_NEG    = 0x48  # Left
CMD_YAW_POS     = 0x49  # Spin counter clockwise
CMD_YAW_NEG     = 0x4A  # Spin clockwise

def strCmd(cmd):
    if CMD_START == cmd:
        return "CMD_START"
    elif CMD_STOP == cmd:
        return "CMD_STOP"
    elif CMD_THRUST_POS == cmd:
        return "CMD_THRUST_POS"
    elif CMD_THRUST_NEG == cmd:
        return "CMD_THRUST_NEG"
    elif CMD_PITCH_POS == cmd:
        return "CMD_PITCH_POS"
    elif CMD_PITCH_NEG == cmd:
        return "CMD_PITCH_NEG"
    elif CMD_ROLL_POS == cmd:
        return "CMD_ROLL_POS"
    elif CMD_ROLL_NEG == cmd:
        return "CMD_ROLL_NEG"
    elif CMD_YAW_POS == cmd:
        return "CMD_YAW_POS"
    elif CMD_YAW_NEG == cmd:
        return "CMD_YAW_NEG"
    else:
        return "INVALID COMMAND"


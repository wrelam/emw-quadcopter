#!/bin/sh
# Starts the appropriate software for interfacing with the XBox controller
rmmod xpad && xboxdrv --silent &

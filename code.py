#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import board
from valuepipe import ValuePipe, Shiftregister
import vp_aoc2b8

import busio
import digitalio
from adafruit_bus_device.spi_device import SPIDevice


# set the local hardware to the
local_hardware = vp_aoc2b8.Analog_Out_2_Channels_8bit()

# set up the bus pins

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

clk_in_pin = digitalio.DigitalInOut(board.GP3)
clk_in_pin.switch_to_input(pull=digitalio.Pull.UP)
data_in_pin = digitalio.DigitalInOut(board.GP4)
data_in_pin.switch_to_input(pull=digitalio.Pull.UP)
latch_in_pin = digitalio.DigitalInOut(board.GP5)
latch_in_pin.switch_to_input(pull=digitalio.Pull.UP)

clk_out_pin = digitalio.DigitalInOut(board.GP6)
clk_out_pin.switch_to_output(drive_mode=digitalio.DriveMode.OPEN_DRAIN)
data_out_pin = digitalio.DigitalInOut(board.GP7)
data_out_pin.switch_to_output(drive_mode=digitalio.DriveMode.OPEN_DRAIN)
latch_out_pin = digitalio.DigitalInOut(board.GP8)
latch_out_pin.switch_to_output(drive_mode=digitalio.DriveMode.OPEN_DRAIN)

vp = ValuePipe(
	clk_in_pin,
	data_in_pin,
	latch_in_pin,
	clk_out_pin,
	data_out_pin,
	latch_out_pin,
	led
)

vp.run(local_hardware)

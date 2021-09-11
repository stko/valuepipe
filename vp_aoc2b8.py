#!/usr/bin/env python
# -*- coding: utf-8 -*-

import board
from valuepipe import SignalBoard, Shiftregister

import busio
import digitalio
from adafruit_bus_device.spi_device import SPIDevice

# the local hardware stuff
# this can certainly moved later into specific python hardware modules


class Analog_Out_2_Channels_8bit(SignalBoard):

	def __init__(self):

		# define the conversion values to write values to outputs
		# value max , output mask multiplier, output mask flags, byte size, endian coding
		self.output_channels = [
			{'max': 0xFF, 'mult': 16, 'flags': 0x1000, 'size': 2, 'endian': 'big'},
			{'max': 0xFF, 'mult': 16, 'flags': 0x9000,
			 'size': 2, 'endian': 'big'}
		]
		self.chip_select_ports = [board.GP13]
		shiftregister = Shiftregister(
			len(self.output_channels) * len(self.chip_select_ports))
		super().__init__(shiftregister)
		# list of available chip select signals
		self.chip_selects = list(map(lambda port: digitalio.DigitalInOut(
			port), self.chip_select_ports))  # assessing the cs ports

		self.spi_bus = busio.SPI(board.GP10, MOSI=board.GP11)
		self.devices = list(map(lambda cs: SPIDevice(
			self.spi_bus, cs), self.chip_selects))  # assessing the cs ports

	def copy_bytearray_into_hardware(self, byte_array):
		''' fills the actual hardware with the corrosponding byte array values
		'''
		value_counter = -1
		for device in self.devices:
			for channel in self.output_channels:
				value_counter += 1
				if value_counter >= len(byte_array):
					break
				# mask out everything > max
				val_out = byte_array[value_counter] & channel['max']
				bytes_out = (channel['flags'] | (
					val_out * channel['mult'])).to_bytes(channel['size'], channel['endian'])
				# The object assigned to spi in the with statements below
				# is the original spi_bus object. We are using the busio.SPI
				# operations busio.SPI.readinto() and busio.SPI.write().
				#bytes_read = bytearray(4)
				# with device as spi:
				#    spi.readinto(bytes_read)
				with device as spi:
					print('SPI write', ''.join('{:02x}'.format(x)
											   for x in bytes_out), val_out)
					spi.write(bytes_out)

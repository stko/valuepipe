#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import supervisor


class ValuePipe:
	'''
	talks between multiple controllers by using GPIOs as serial ring bus, handle commands coming from the serial terminal
	'''
	max_bytes_of_byte_field = 8 * \
		256  # if the initialisation of the byte field is not finished after this, show an error

	def __init__(self,
				 clk_in_pin,
				 data_in_pin,
				 latch_in_pin,
				 clk_out_pin,
				 data_out_pin,
				 latch_out_pin,
				 led=None
				 ) -> None:
		self.clk_in_pin = clk_in_pin
		self.data_in_pin = data_in_pin
		self.latch_in_pin = latch_in_pin
		self.clk_out_pin = clk_out_pin
		self.data_out_pin = data_out_pin
		self.latch_out_pin = latch_out_pin
		self.led = led

	# the serial protocol stuff

	def signal_pin(self, pin, value):
		pin.value = value

	def wait_for_correct_signal_level(self, pin, level, pin_name):
		err_trials = 1000
		while pin.value != level:
			time.sleep(0.001)
			err_trials -= 1
			if not err_trials:  # to many trials, error
				print('Signal error: {0} has permanent {1}'.format(
					pin_name, not level))
				return False
		# print('correct level: {0} has  {1} with err_trials {2}'.format(pin_name, level,err_trials))
		return True

	def clk_out(self, value, controller):
		self.signal_pin(self.clk_out_pin, value)
		if controller:
			self.wait_for_correct_signal_level(self.clk_in_pin, value, 'clk')

	def clk_high(self, controller=False):
		self.clk_out(True, controller)

	def clk_low(self, controller=False):
		self.clk_out(False, controller)

	def data_out(self, value, controller):
		if self.led:
			self.led.value = value
		self.signal_pin(self.data_out_pin, value)
		# wait_for_correct_signal_level(data_in_pin, value, 'data')

	def data_high(self, controller=False):
		self.data_out(True, controller)

	def data_low(self, controller=False):
		self.data_out(False, controller)

	def latch_out(self, value, controller):
		self.signal_pin(self.latch_out_pin, value)
		if controller:
			self.wait_for_correct_signal_level(
				self.latch_in_pin, value, 'latch')

	def latch_high(self, controller=False):
		self.latch_out(True, controller)

	def latch_low(self, controller=False):
		self.latch_out(False, controller)

	def clk(self):
		return self.clk_in_pin.value

	def data(self):
		return self.data_in_pin.value

	def latch(self):
		return self.latch_in_pin.value

	def bus_down(self):
		# just a helper routine to switch off all bus signals, if actual not used
		self.latch_low()
		self.clk_low()
		self.data_low()

	def init_master(self):
		print("init command detected, starting Master initialisation")
		# play the reset sequence by set clk and latch at the same time
		self.clk_high(True)
		self.latch_high(True)
		self.latch_low(True)
		self.clk_low(True)
		self.data_high(True)  # set the data line on high
		tick_counter = 0
		while not self.data() and tick_counter < self.max_bytes_of_byte_field:
			self.clk_high(True)
			self.clk_low(True)
			tick_counter += 1
		print('tick_counter', tick_counter)

		self.bus_down()  # switch the bus signals off
		return tick_counter - 1

	def send_bit(self, bit):
		if bit:
			self.data_high(True)
		else:
			self.data_low(True)
		self.clk_high(True)
		self.clk_low(True)

	def send_byte(self, byte_out):
		for i in range(8):
			self.send_bit(byte_out & 0x80)  # highest bit set?
			byte_out = byte_out << 1  # next bit

	def run(self, local_hardware):
		print('Start- please remember that the first input line can contain garbage...')
		input_chars = ''
		clk_in_signal_edge_detection = False
		data_in_signal_edge_detection = False
		latch_in_signal_edge_detection = False
		data_out_signal = False
		while True:
			# read the input signals as Bool
			clk_in_signal = self.clk()
			data_in_signal = self.data()
			latch_in_signal = self.latch()

			# edge detection
			clk_in_signal_pos_edge = clk_in_signal and not clk_in_signal_edge_detection
			data_in_signal_pos_edge = data_in_signal and not data_in_signal_edge_detection
			latch_in_signal_pos_edge = latch_in_signal and not latch_in_signal_edge_detection

			# store signals
			clk_in_signal_edge_detection = clk_in_signal
			data_in_signal_edge_detection = data_in_signal
			latch_in_signal_edge_detection = latch_in_signal

			# reset command?
			if clk_in_signal and latch_in_signal_pos_edge:  # reset
				print("Reset")
				local_hardware.reset()
				continue

			# prepare an output

			if clk_in_signal_pos_edge:
				data_out_signal = local_hardware.shiftregister.shift(
					data_in_signal)

			if latch_in_signal_pos_edge:
				local_hardware.latch()

			# passthrough the bits to the output, if permitted
			# set the data pin first
			#print('set data_out_signal', data_out_signal)
			self.data_out(data_out_signal, False)
			self.clk_out(clk_in_signal, False)
			self.latch_out(latch_in_signal, False)

			# handle serial input in case we are the master
			nr_of_chars = supervisor.runtime.serial_bytes_available
			if nr_of_chars:
				input_chars += sys.stdin.read(nr_of_chars)
				while '\n' in input_chars:
					local_hardware_bytearray = local_hardware.dataarray()
					line = input_chars[:input_chars.find('\n')+1]
					input_chars = input_chars[input_chars.find('\n')+1:]
					line = line.lower().strip()
					if line == 'init':
						self.init_master()
					values = line.split()
					for value_counter in range(len(values)):
						this_val = values[value_counter].strip()
						try:
							val_out = int(this_val)
							val_out &= 255  # mask out everything > 255
						except:
							val_out = 0
						if value_counter >= len(local_hardware_bytearray):
							self.send_byte(val_out)
						else:
							local_hardware_bytearray[value_counter] = val_out
					self.latch_high(True)
					self.latch_low(True)
					self.bus_down()  # switch the bus signals off after a transfer sequence
					local_hardware.copy_bytearray_into_hardware(
						local_hardware_bytearray)
			else:
				time.sleep(0.001)


class Shiftregister:
	# a software shift register,
	# the initial idea was to make this as a child class of bytearray,
	# but that crashes in circuitpython (bug?)
	# so I implemented the bytearray as member, but not as superclass

	def __init__(self, size):
		self.ba = bytearray(size)

	def shift(self, bit_in):
		'''
		put the boolean in_bit into the bit chain as the LSB, returns the MSB as boolean
		'''
		for b in range(len(self.ba)):
			bit_out = self.ba[b] & 0x80 != 0
			if bit_in:
				self.ba[b] = ((self.ba[b] << 1) & 0xFF) + 1
			else:
				self.ba[b] = (self.ba[b] << 1) & 0xFF
			bit_in = bit_out
		return bit_out

	def fill(self, input_array):
		'''
		fills its bytearray with the data from input_array
		'''
		self.ba = bytearray(input_array)

	def get(self):
		'''
		returns bytearray with the data from self
		'''
		return bytearray(self.ba)

	def __len__(self):
		return len(self.ba)


# the generic Hardware Board class which can be managed by valuepipe
class SignalBoard:

	def __init__(self, shiftregister):
		self.shiftregister = shiftregister

		# define the conversion values to write values to outputs
		# value max , output mask multiplier, output mask flags, byte size, endian coding
		print('Error: Board() is not overloaded')

	def reset(self):
		self.shiftregister = Shiftregister(len(self.shiftregister))

	def dataarray(self):
		''' returns a byte array representing the I/Os of this hardware
		'''
		return self.shiftregister.get()

	def latch(self):
		self.copy_bytearray_into_hardware(self.dataarray())
		print('actual bytes', self.dataarray())

	def copy_hardware_into_bytearray(self, byte_array):
		''' fills the actual hardware values into the corrosponding byte array
		'''

	def copy_bytearray_into_hardware(self, byte_array):
		''' fills the actual hardware with the corrosponding byte array values
		'''
		print('Error: Board.copy_bytearray_into_hardware is not overloaded')

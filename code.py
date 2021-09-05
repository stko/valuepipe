import sys
import time
import board
import supervisor

import busio
import digitalio
from adafruit_bus_device.spi_device import SPIDevice

# a software shift register,
# the initial idea was to make this as a child class of bytearray,
# but that crashes in circuitpython (bug?)
# so I implemented the bytearray as member, but not as superclass

max_bytes_of_byte_field= 8 * 256 # if the initialisation of the byte field is not finished after this, show an error

class Shiftregister:

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


# the local hardware stuff
# this can certainly moved later into specific python hardware modules
class Analog_Out_2_Channels_8bit:

	def __init__(self):

		# define the conversion values to write values to outputs
		# value max , output mask multiplier, output mask flags, byte size, endian coding
		self.output_channels = [
			{'max': 0xFF, 'mult': 16, 'flags': 0x1000, 'size': 2, 'endian': 'big'},
			{'max': 0xFF, 'mult': 16, 'flags': 0x9000,
			 'size': 2, 'endian': 'big'}
		]
		self.chip_select_ports = [board.GP13]
		self.shiftregister = Shiftregister(
			len(self.output_channels) * len(self.chip_select_ports))
		# list of available chip select signals
		self.chip_selects = list(map(lambda port: digitalio.DigitalInOut(
			port), self.chip_select_ports))  # assessing the cs ports

		self.spi_bus = busio.SPI(board.GP10, MOSI=board.GP11)
		self.devices = list(map(lambda cs: SPIDevice(
			self.spi_bus, cs), self.chip_selects))  # assessing the cs ports

	def reset(self):
		self.shiftregister = Shiftregister(len(self.shiftregister))

	def dataarray(self):
		''' returns a byte array representing the I/Os of this hardware
		'''
		return self.shiftregister.get()

	def latch(self):
		self.copy_bytearray_into_hardware(self.dataarray())
		print('actual bytes',self.dataarray())

	def copy_hardware_into_bytearray(self, byte_array):
		''' fills the actual hardware values into the corrosponding byte array
		'''

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


# the serial protocol stuff


def signal_pin(pin, value):
	pin.value = value


def wait_for_correct_signal_level(pin, level, pin_name):
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


def clk_out(value, controller):
	signal_pin(clk_out_pin, value)
	if controller:
		wait_for_correct_signal_level(clk_in_pin, value, 'clk')


def clk_high(controller=False):
	clk_out(True, controller)


def clk_low(controller=False):
	clk_out(False, controller)


def data_out(value, controller):
	led.value = value
	signal_pin(data_out_pin, value)
	# wait_for_correct_signal_level(data_in_pin, value, 'data')


def data_high(controller=False):
	data_out(True, controller)


def data_low(controller=False):
	data_out(False, controller)


def latch_out(value, controller):
	signal_pin(latch_out_pin, value)
	if controller:
		wait_for_correct_signal_level(latch_in_pin, value, 'latch')


def latch_high(controller=False):
	latch_out(True, controller)


def latch_low(controller=False):
	latch_out(False, controller)


def clk():
	return clk_in_pin.value


def data():
	return data_in_pin.value


def latch():
	return latch_in_pin.value


def bus_down():
	# just a helper routine to switch off all bus signals, if actual not used
	latch_low()
	clk_low()
	data_low()


def init_master():
	print("init command detected, starting Master initialisation")
	# play the reset sequence by set clk and latch at the same time
	clk_high(True)
	latch_high(True)
	latch_low(True)
	clk_low(True)
	data_high(True)  # set the data line on high
	tick_counter = 0
	while not data() and tick_counter < max_bytes_of_byte_field:
		clk_high(True)
		clk_low(True)
		tick_counter += 1
	print('tick_counter', tick_counter)

	bus_down()  # switch the bus signals off
	return tick_counter -1


def send_bit(bit):
	if bit:
		data_high(True)
	else:
		data_low(True)
	clk_high(True)
	clk_low(True)


def send_byte(byte_out):
	for i in range(8):
		send_bit(byte_out & 0x80)  # highest bit set?
		byte_out = byte_out << 1  # next bit


# set the local hardware to the
local_hardware = Analog_Out_2_Channels_8bit()

#

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

print('Start- please remember that the first input line can contain garbage...')
input_chars = ''
clk_in_signal_edge_detection = False
data_in_signal_edge_detection = False
latch_in_signal_edge_detection = False
data_out_signal = False
while True:
	# read the input signals as Bool
	clk_in_signal = clk()
	data_in_signal = data()
	latch_in_signal = latch()

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
		data_out_signal = local_hardware.shiftregister.shift(data_in_signal)

	if latch_in_signal_pos_edge:
		local_hardware.latch()

	# passthrough the bits to the output, if permitted
	# set the data pin first
	#print('set data_out_signal', data_out_signal)
	data_out(data_out_signal, False)
	clk_out(clk_in_signal, False)
	latch_out(latch_in_signal, False)

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
				init_master()
			values = line.split()
			for value_counter in range(len(values)):
				this_val = values[value_counter].strip()
				try:
					val_out = int(this_val)
					val_out &= 255  # mask out everything > 255
				except:
					val_out = 0
				if value_counter >= len(local_hardware_bytearray):
					send_byte(val_out)
				else:
					local_hardware_bytearray[value_counter] = val_out
			latch_high(True)
			latch_low(True)
			bus_down()  # switch the bus signals off after a transfer sequence
			local_hardware.copy_bytearray_into_hardware(
				local_hardware_bytearray)
	else:
		time.sleep(0.001)

import sys
import time
import board
import supervisor

import busio
import digitalio

#####################  the local hardware stuff
# this can certainly moved later into specific python hardware modules
class Analog_Out_2_Channels_8bit:
    from adafruit_bus_device.spi_device import SPIDevice

    def __init__(self):
        
        ''' returns a byte array representing the I/Os of this hardware
        '''
        self.bytearray=bytearray([0,0])
 
    def bytearray():
        return self.bytearray.copy()
        
    def copy_hardware_into_bytearray(self, byte_array):
        ''' fills the actual hardware values into the corrosponding byte array
        '''
    import sys
import time
import board
import supervisor

import busio
import digitalio

#####################  the local hardware stuff
# this can certainly moved later into specific python hardware modules
class Analog_Out_2_Channels_8bit:
    from adafruit_bus_device.spi_device import SPIDevice

    def __init__(self):
        
        ''' returns a byte array representing the I/Os of this hardware
        '''
        self.bytearray=bytearray([0,0])
        # define the conversion values to write values to outputs
        # value max , output mask multiplier, output mask flags, byte size, endian coding
        self.output_channels =[
            {'max':0xFF , 'mult':16 , 'flags' : 0x1000 , 'size' : 2 , 'endian' : 'big' },
            {'max':0xFF , 'mult':16 , 'flags' : 0x9000 , 'size' : 2 , 'endian' : 'big' }
        ]

 
    def dataarray(self):
        return bytearray(self.bytearray)
        
    def copy_hardware_into_bytearray(self, byte_array):
        ''' fills the actual hardware values into the corrosponding byte array
        '''
    

#####################  the serial protocol stuff


def signal_pin(pin, value):
    pin.value=value

def wait_for_correct_signal_level(pin,level):
    err_trials=10
    while pin.value!=level:
        time.sleep(0.001)
        err_trials-=1
        if not err_trials: # to many trials, error
            print('Signal error: {0} has permanent {1}'.format(pin.name,not level))
            return False
    print('correct level: {0} has  {1}'.format(str(pin), level))
    return True
    
def clk_high():
    signal_pin(clk_out_pin, True)
    wait_for_correct_signal_level(clk_out_pin,True)

def data_high():
    signal_pin(data_out_pin, True)
    wait_for_correct_signal_level(data_out_pin,True)

def latch_high():
    signal_pin(latch_out_pin, True)
    wait_for_correct_signal_level(latch_out_pin,True)

def clk_low():
    signal_pin(clk_out_pin, False)
    wait_for_correct_signal_level(clk_out_pin,False)

def data_low():
    signal_pin(data_out_pin, False)
    wait_for_correct_signal_level(data_out_pin,False)

def latch_low():
    signal_pin(latch_out_pin, False)
    wait_for_correct_signal_level(latch_out_pin,False)

def init_master():
    # play the reset sequence by set clk and latch at the same time
    clk_high()
    latch_high()
    latch_low()
    clk_low()

# set the local hardware to the 
local_hardware=Analog_Out_2_Channels_8bit()
local_hardware_bytearray=local_hardware.dataarray()
def reset():
    for i in range(len(local_hardware_bytearray)):
        local_hardware_bytearray[i]=0
        print('bla')
        pass

# list of available chip select signals
chip_select_ports=[board.GP13]

#
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


if supervisor.runtime.serial_connected:
    print("Serial Input detected, starting Master initialisation")
    init_master()
    reset()


chip_selects=list(map(lambda port: digitalio.DigitalInOut(port), chip_select_ports)) # assessing the cs ports

with busio.SPI(board.GP10, MOSI=board.GP11) as spi_bus:
    devices = list(map(lambda cs: SPIDevice(spi_bus, cs), chip_selects)) # assessing the cs ports
    print('Start- please remember that the first input line can contain garbage...')
    input_chars=''
    while False:
        # read the input signals as Bool

        clk_in_signal=clk_in_pin.value
        data_in_signal=data_in_pin.value
        latch_in_signal=latch_in_pin.value
        
        clk_out_pin.value=clk_in_signal
        data_out_pin.value=data_in_signal
        latch_out_pin.value=latch_in_signal 
        
        if clk_in_signal and latch_in_signal: #reset
            reset()
            continue
        

        # handle serial input in case we are the master
        nr_of_chars=supervisor.runtime.serial_bytes_available
        if nr_of_chars:
            input_chars +=sys.stdin.read(nr_of_chars)
            while '\n' in input_chars:
                line=input_chars[:input_chars.find('\n')+1]
                input_chars=input_chars[input_chars.find('\n')+1:]
                line=line.strip()
                values=line.split()
                value_counter=-1
                for device in devices:
                    for channel in output_channels:
                        value_counter += 1
                        if value_counter>= len(values):
                            break
                        this_val=values[ value_counter ].strip()
                        try:
                            val_out = int( this_val )
                            val_out  &= channel['max'] # mask out everything > max
                        except:
                            val_out=0

                        bytes_out = (channel['flags'] | (val_out * channel['mult'] ) ).to_bytes(channel['size'],channel['endian'])
                        # The object assigned to spi in the with statements below
                        # is the original spi_bus object. We are using the busio.SPI
                        # operations busio.SPI.readinto() and busio.SPI.write().
                        #bytes_read = bytearray(4)
                        #with device as spi:
                        #    spi.readinto(bytes_read)
                        with device as spi:
                            print('send', ''.join('{:02x}'.format(x) for x in bytes_out), val_out)
                            spi.write(bytes_out)

#####################  the serial protocol stuff


def signal_pin(pin, value):
    pin.value=value

def wait_for_correct_signal_level(pin,level):
    err_trials=10
    while pin.value!=level:
        time.sleep(0.001)
        err_trials-=1
        if not err_trials: # to many trials, error
            print('Signal error: {0} has permanent {1}'.format(pin.name,not level))
            return False
    print('correct level: {0} has  {1}'.format(str(pin), level))
    return True
    
def clk_high():
    signal_pin(clk_out_pin, True)
    wait_for_correct_signal_level(clk_out_pin,True)

def data_high():
    signal_pin(data_out_pin, True)
    wait_for_correct_signal_level(data_out_pin,True)

def latch_high():
    signal_pin(latch_out_pin, True)
    wait_for_correct_signal_level(latch_out_pin,True)

def clk_low():
    signal_pin(clk_out_pin, False)
    wait_for_correct_signal_level(clk_out_pin,False)

def data_low():
    signal_pin(data_out_pin, False)
    wait_for_correct_signal_level(data_out_pin,False)

def latch_low():
    signal_pin(latch_out_pin, False)
    wait_for_correct_signal_level(latch_out_pin,False)

def init_master():
    # play the reset sequence by set clk and latch at the same time
    clk_high()
    latch_high()
    latch_low()
    clk_low()

# set the local hardware to the 
local_hardware=Analog_Out_2_Channels_8bit()
local_hardware_bytearray=local_hardware.bytearray()
def reset():
    for i in range(len(local_hardware_bytearray)):
        local_hardware_bytearray[i]=0
        print('bla')
        pass

# list of available chip select signals
chip_select_ports=[board.GP13]

#
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


if supervisor.runtime.serial_connected:
    print("Serial Input detected, starting Master initialisation")
    init_master()
    reset()

# define the conversion values to write values to outputs
# value max , output mask multiplier, output mask flags, byte size, endian coding
output_channels =[
    {'max':0xFF , 'mult':16 , 'flags' : 0x1000 , 'size' : 2 , 'endian' : 'big' },
    {'max':0xFF , 'mult':16 , 'flags' : 0x9000 , 'size' : 2 , 'endian' : 'big' }
]

chip_selects=list(map(lambda port: digitalio.DigitalInOut(port), chip_select_ports)) # assessing the cs ports

with busio.SPI(board.GP10, MOSI=board.GP11) as spi_bus:
    devices = list(map(lambda cs: SPIDevice(spi_bus, cs), chip_selects)) # assessing the cs ports
    print('Start- please remember that the first input line can contain garbage...')
    input_chars=''
    while False:
        # read the input signals as Bool

        clk_in_signal=clk_in_pin.value
        data_in_signal=data_in_pin.value
        latch_in_signal=latch_in_pin.value
        
        clk_out_pin.value=clk_in_signal
        data_out_pin.value=data_in_signal
        latch_out_pin.value=latch_in_signal 
        
        if clk_in_signal and latch_in_signal: #reset
            reset()
            continue
        

        # handle serial input in case we are the master
        nr_of_chars=supervisor.runtime.serial_bytes_available
        if nr_of_chars:
            input_chars +=sys.stdin.read(nr_of_chars)
            while '\n' in input_chars:
                line=input_chars[:input_chars.find('\n')+1]
                input_chars=input_chars[input_chars.find('\n')+1:]
                line=line.strip()
                values=line.split()
                value_counter=-1
                for device in devices:
                    for channel in output_channels:
                        value_counter += 1
                        if value_counter>= len(values):
                            break
                        this_val=values[ value_counter ].strip()
                        try:
                            val_out = int( this_val )
                            val_out  &= channel['max'] # mask out everything > max
                        except:
                            val_out=0

                        bytes_out = (channel['flags'] | (val_out * channel['mult'] ) ).to_bytes(channel['size'],channel['endian'])
                        # The object assigned to spi in the with statements below
                        # is the original spi_bus object. We are using the busio.SPI
                        # operations busio.SPI.readinto() and busio.SPI.write().
                        #bytes_read = bytearray(4)
                        #with device as spi:
                        #    spi.readinto(bytes_read)
                        with device as spi:
                            print('send', ''.join('{:02x}'.format(x) for x in bytes_out), val_out)
                            spi.write(bytes_out)

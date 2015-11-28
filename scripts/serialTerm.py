import serial
from random import randint
from time import sleep

print '\n' + '*'*80
print 'Test that generates 12 random 8-bit ints and writes to EEPROM.'
print 'It then reads the EEPROM and outputs the resulting values to be'
print 'compared with a previous EEPROM state.'
print '*'*80

port = '/dev/ttyACM0'
baud = 115200

ser = serial.Serial(port, baud, timeout=1)
ser.close()
ser.open()
#ser.flush()
sleep(2)    # It doesn't work OK w/o this
"""
With no delay (sleep) only requests are sent via serial. No response is read.
With a delay of 1 s or less, two bytes are read as response in the middle of the
communication:

RX Pin   W | " " |  X |  Y |  Z | " " |  A | ...
TX Pin     |     | 20 | 16 

Both bytes have always the same value, 20 and 16.
"""


###################################
def r(sz):
    read_data = ser.read(size=sz)
    print read_data
###################################
def w(data):
    for d in data:
        ser.write(d)
###################################
def writeRandom ():
    ra = 'W'
    data = ''
    for k in range(12):
        data += ' ' + (str(randint(0, 255)))
    ra += data + '\r'
    # Test data to write
    print '\nRandom data:', data
    w(ra); #sleep(1)
    print 'Writing random data to EEPROM...'
    r(3)
###################################
def readEEPROM ():
    # Read EEPROM
    print '\nRead EEPROM state:'
    w("R\r")
    
    r(50)
###################################

writeRandom()

readEEPROM()

ser.close()
print '*' * 80

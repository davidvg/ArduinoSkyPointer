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
ser.flush()

sleep(.1)


def r(sz):
    read_data = ser.read(size=sz)
    print read_data

def w(data):
    for d in data:
        ser.write(d)
    
# Initial state of EEPROM
w("R\r")
print 'EEPROM state:'
r(60)

ra = 'W'
data = ''
for k in range(12):
    data  = data + ' ' + (str(randint(0, 255)))
ra = ra + data + '\r'

# Test data to write
print '\nRandom data:', data
w(ra)
print '\nWriting data to EEPROM...'
r(3)

# Read EEPROM
print '\nNew EEPROM state:'
w("R\r")
r(60)

print '*'*80

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
#ser.close()
#ser.open()
#ser.flush()
sleep(3)


###################################
def r(sz):
    read_data = ser.read(size=sz)
    print read_data
###################################
def w(data):
    for d in data:
        ser.write(d)
###################################
#ra = 'W'
#for k in range(12):
#    ra += ' ' + (str(randint(0, 255)))
#ra += '\r'
#print ra
#w(ra)
#sleep(.5)
#r(3)
    
#print 'EEPROM state:'
#w("R\r")
#r(50)

#------------------------------------------------------------#
# It doesn't work without this!
r(50)
#------------------------------------------------------------#

ra = 'W'
data = ''
for k in range(12):
    data  = data + ' ' + (str(randint(0, 255)))
ra = ra + data + '\r'
# Test data to write
print '\nRandom data:', data
w(ra); sleep(1)
print 'Writing random data to EEPROM...'
r(3)

# Read EEPROM
print '\nRead EEPROM state:'
w("R\r")
r(50)

#ser.flush(); sleep(1)
ser.close()
print '*' * 80

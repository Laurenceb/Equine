import serial
import time

ser=serial.Serial("/dev/ttyUSB0",460800,timeout=1)

while 1:
	time.sleep(0.007)
	ser.write("1234567890123456789012345678901234567890123456789012345678901234")#64bytes

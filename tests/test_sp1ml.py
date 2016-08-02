import serial
import time

ser=serial.Serial("/dev/ttyUSB0",115200,timeout=1)


ser.setRTS(False)
time.sleep(0.2)
ser.write("ATS01?\n\r")
print(ser.readline())
ser.setRTS(True)




import serial
import time
ser = serial.Serial('/dev/ttyUSB0',9600)  # open serial port
print(ser.name)         # check which port was really used
#time.sleep(1.0)

while True:
    ser.write('B0\n')     # write a string
    time.sleep(0.01)
#ser.flush() 
#time.sleep(1.0)
ser.close()             # close port

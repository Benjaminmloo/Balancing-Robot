import serial
import sys
import time

ser = serial.Serial('/dev/ttyACM0',9600)
ser.write('a') 
time.sleep(0.5)
ser.write('e') 
sys.exit()

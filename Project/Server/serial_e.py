import serial
import sys

ser = serial.Serial('/dev/ttyACM0',9600)
ser.write('e') 
sys.exit()

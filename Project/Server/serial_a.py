import serial
import sys
#EDIT
ser = serial.Serial('/dev/ttyACM0',9600)
ser.write('a') 
sys.exit()

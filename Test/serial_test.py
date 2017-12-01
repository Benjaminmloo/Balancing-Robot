import serial
import thread
import struct
import time

ser = serial.Serial('/dev/ttyACM0',9600)
file = open("log.csv", "w");


def read():

        while True:
                read_serial=ser.readline()
                file.write(read_serial);
                #time.sleep(1)
                print (read_serial)
def write():
        count = 0
        while True:
                if count < 200:
                        ser.write(struct.pack(">B",0))
                elif count < 400:
                        ser.write(struct.pack(">B",10))
                else:
                        count = 0
                count += 1
                #print count

try:
        #thread.start_new_thread(write, ())
        thread.start_new_thread(read, ())
except:
        print "Error creating threads"


while 1:
        pass

#!/usr/bin/env python
from read_vn300 import gps;
import serial;
import time;

#reset baud
baud = 115200;

imu_baud = 921600;

g = gps(dev='/dev/ttyUSB0',baudrate=baud,timeout=2);
g.reset(True);

print "Async mode before: %s"%g.write_register(6);

g.async_off();

print "Async mode after: %s"%g.write_register(6);

time.sleep(2);
print "Current baud before: %s"%g.write_register(5);

g.write_register(5, imu_baud);

g.ser.close();


g.ser = serial.Serial('/dev/ttyUSB0',
                      timeout=2,baudrate=imu_baud, bytesize=8, parity='N',
                      stopbits=1)

resp="ERR";
while ('ERR' in resp):
    resp=g.write_register(5, imu_baud); # write to baud rate reg as sec arg
    time.sleep(1);

if(str(imu_baud) in resp):
    print "Success, ready for 921kHz";
    print resp
g.ser.close();


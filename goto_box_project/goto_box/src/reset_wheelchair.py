#!/usr/bin/env python

import serial
port1='/dev/goto_box_usb' #adjust this in case your device differs
ser1 = serial.Serial() #open serial connection
ser1.port=port1 #setting the port accordingly
 
#in case your usual baudrate isn't 9600 reset will not work, therefore we will open a resetable connection
#thanks to mattvenn.net for suggesting to add this step!
ser1.baudrate=57600
ser1.open(); ser1.close()
 
ser1.baudrate=1200 # set the reset baudrate
ser1.open(); ser1.close()
#don't forget to sleep some reset time

port2='/dev/wheelchair_mega' 
ser2 = serial.Serial()
ser2.port=port2

ser2.baudrate=115200
ser2.open(); ser2.close()

ser2.baudrate=1200
ser2.open(); ser2.close()

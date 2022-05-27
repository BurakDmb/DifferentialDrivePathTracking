#!/usr/bin/python3

import serial
import os
import time
import gps_driver_py3 as gps

ser=gps.init_line()
for i in range(500):
    val=gps.read_gll(ser,20)
    #print(val)
    #Latitude=(int(str(val[0])[0:2])*3600+float(str(val[0])[2:])*60)/3600
    LatD=int((str(val[0]).split('.'))[0][:-2])
    LatM=float(((str(val[0]).split('.'))[0][-2:])+'.'+((str(val[0]).split('.'))[1]))
    Latitude=round((LatD*3600+LatM*60)/3600,7)
    
    LonD=int((str(val[2]).split('.'))[0][:-2])
    LonM=float(((str(val[2]).split('.'))[0][-2:])+'.'+((str(val[2]).split('.'))[1]))
    Longitude=round((LonD*3600+LonM*60)/3600,7)

    #print(Latitude)
    #print(Longitude)
    #print(LM)
    #time.sleep(1.0)
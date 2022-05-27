import serial
import os
import time

def init_line():
    ser = serial.Serial('/dev/ttyS0',timeout=1.0)
    time.sleep(1.0)
    #print(ser)
    return ser

def read_gll(ser,nmax=20):
    val=[0.,'N',0.,'W',0.]
    for i in range(nmax):
        v=ser.readline().decode("utf-8")
        #print(v)
        if str(v[0:6]) == "$GNGLL":
            vv = v.split(",")
            #print(vv)
            val[0] = float(vv[1])
            #print(val[0])
            val[1] = vv[2]
            #print(val[1])
            val[2] = float(vv[3])
            #print(val[2])
            val[3] = vv[4]
            #print(val[3])
            val[4] = float(vv[5])
            #print(val[4])
        
            break
    return val
   

def close(ser):
    ser.close()

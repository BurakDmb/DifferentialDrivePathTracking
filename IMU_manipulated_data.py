import serial
import os
import time
import numpy as np


def init_line():
    ser = serial.Serial('/dev/ttyUSB0',baudrate=9600,timeout=0.1)
    ser.write(('#ox').encode("utf-8"))
    time.sleep(1.0)
    
    ser.write(('#osct').encode("utf-8")) 
    for x in range(50):
        val = read_gll(ser)
        print(val)
    print("Heeeey")   
    return ser

def read_gll(ser):         
    ser.flushInput()
    ser.flushOutput()
    data = ser.readline()
    v = str(data, 'UTF-8') 
    return v
    
def split_string(raw_data):
    clean_data1 = raw_data[5:]
    clean_data2 = clean_data1[:len(clean_data1)-2]
    clean_data = clean_data2.split(",")
    readings = []
    for data in clean_data:
        readings.append(float(data))
    return readings

def IMU_readings(all_readings):
    att_readings = np.array([all_readings[0]])
#     att_readings = np.array([all_readings[0], all_readings[1], all_readings[2]])
    #linacc_readings = np.array([all_readings[3], all_readings[4], all_readings[5]])
    #rotvel_readings = np.array([all_readings[6], all_readings[7], all_readings[8]])
    return att_readings#, linacc_readings, rotvel_readings
    
if __name__ == "__main__":
    ser = init_line()
    #ser.write(('#ox').encode("utf-8")) 
    while True:
        val = read_gll(ser)
        print("val, ",val)
        if val[5]=="G":
            float_readings = split_string(val)
            Attitude_values, LinAcc_values, RotVel_values = IMU_readings(float_readings)
            print(Attitude_values, (9.8/256)*LinAcc_values, RotVel_values)
            #Attitude is in degrees(not so sure about this)
            #Linear acceleration is in m/s^2
            #Rotational velocity is in rad/s
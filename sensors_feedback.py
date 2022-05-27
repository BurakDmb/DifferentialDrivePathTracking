import serial
import os
import time
import IMU_manipulated_data as imu
import gps_driver_py3 as gps

INIT_ANGLE = 0

def initialization():
    global INIT_ANGLE
    ser_gps = gps.init_line()
    ser_imu = imu.init_line()
    
    INIT_ANGLE = get_heading_angle(ser_imu)
    return ser_gps, ser_imu
    

def get_gps_coordinates(ser):
    ser.flushInput()
    ser.flushOutput()
    val=gps.read_gll(ser,20)
    #Latitude=(int(str(val[0])[0:2])*3600+float(str(val[0])[2:])*60)/3600
    LatD=int((str(val[0]).split('.'))[0][:-2])
    LatM=float(((str(val[0]).split('.'))[0][-2:])+'.'+((str(val[0]).split('.'))[1]))
    Latitude=round((LatD*3600+LatM*60)/3600,7)
    
    LonD=int((str(val[2]).split('.'))[0][:-2])
    LonM=float(((str(val[2]).split('.'))[0][-2:])+'.'+((str(val[2]).split('.'))[1]))
    Longitude=round((LonD*3600+LonM*60)/3600,7)
    
    return Latitude, Longitude

def get_heading_angle(ser):
    #print(ser)
    global INIT_ANGLE
    val = imu.read_gll(ser)
    
        
    
#     float_readings = imu.split_string(val)
#     print("After split",float_readings)
#     Attitude_values = imu.IMU_readings(float_readings)
    Attitude_values = float(val)
    #print(Attitude_values)
    #print(Attitude_values, (9.8/256)*LinAcc_values, RotVel_values)
#     return (Attitude_values[0] - INIT_ANGLE)
    return (Attitude_values)

def get_sensors_data(ser_gps,ser_imu):
    
    lat, lon = get_gps_coordinates(ser_gps)
    angle = get_heading_angle(ser_imu)
    
    return lat,lon,angle
    
    


    

    



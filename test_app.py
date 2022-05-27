import serial
import os
import time
import numpy as np
import sensors_feedback as sensors


if __name__ == "__main__":
    ser_gps, ser_imu = sensors.initialization()
    
    while True:
        #lat , long = sensors.get_gps_coordinates(ser_gps)
        heading_angle = sensors.get_heading_angle(ser_imu)
        print("heading_angle = ",heading_angle)
        #print("Lat %d Longf %d ",lat,long)
        lat,lon,heading = get_sensors_data(ser_gps,ser_imu)
        
        
        
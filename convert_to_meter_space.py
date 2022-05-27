import numpy as np
import math
def convert_to_meter_space(lon, lat):
    k = 6378137
    x = lon * (k * np.pi/180.0)
    y = np.log(np.tan((90 + lat) * np.pi/360.0)) * k
    #print(x)
    #print(y)
    return x,y
ylGPS,xlGPS=43.133200, 6.006932
yuGPS,xuGPS=43.138136, 6.023519

x1,y1=convert_to_meter_space(xlGPS, ylGPS)
x2,y2=convert_to_meter_space(xuGPS, yuGPS)

print(f"{x1},{y1}")

print(f"{x2},{y2}")


print(f"{x2-x1},{y2-y1}")
print(f"{math.sqrt((x2-x1)**2-(y2-y1)**2)}")
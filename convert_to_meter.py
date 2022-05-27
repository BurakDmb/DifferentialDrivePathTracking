
import math
import matplotlib.image as mpimg

from math import pi, sin, cos, atan2,atan, sqrt
ylGPS,xlGPS=43.133200, 6.006932
yuGPS,xuGPS=43.138136, 6.023519

def convert_to_meter_coord(lat1, lon1, lat2, lon2, img_width, img_height):
    R = 6378.137 # Radius of earth in KM
    dLat = lat2 * pi / 180 - lat1 * pi / 180
    dLon = lon2 * pi / 180 - lon1 * pi / 180
    a = sin(dLat/2) * sin(dLat/2) +cos(lat1 * pi / 180) * cos(lat2 * pi / 180) *sin(dLon/2) * sin(dLon/2)
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    d=d*1000 # % meters
    angle=atan(img_height/img_width)
    print(d)
    #print(x)
    #print(y)
    #return x, y
im =mpimg.imread("map.png")
# <class 'numpy.ndarray'>
xl=0
yl=0
img_width=im.shape[1]
img_height=im.shape[0]
print(img_width)
print(img_height)
convert_to_meter_coord(ylGPS, xlGPS, yuGPS, xuGPS, img_width, img_height)
from get_points_from_map import get_points_from_map

import matplotlib.pyplot as plt 
import matplotlib.image as mpimg


def convert_points2GPS(xp,yp,xlGPS,ylGPS,xuGPS,yuGPS):
    im =mpimg.imread("map.png")
    # <class 'numpy.ndarray'>
    xl=0
    yl=0
    xu=im.shape[1]
    yu=im.shape[0]
    xMapped=((xp-xl)*(xuGPS-xlGPS))/(xu-xl)+xlGPS
    yMapped=((yp-yl)*(yuGPS-ylGPS))/(yu-yl)+ylGPS
    return xMapped,yMapped
points=get_points_from_map()
xp=points[0][0]
yp=points[0][1]
#xlGPS,ylGPS=-10,-5
#xuGPS,yuGPS=10,5
xlGPS,ylGPS=-3.026018,48.207114
xuGPS,yuGPS=-3.009049,48.213114
xMapped,yMapped=convert_points2GPS(xp,yp,xlGPS,ylGPS,xuGPS,yuGPS)
print(f"{xMapped},{yMapped}")
print(points)
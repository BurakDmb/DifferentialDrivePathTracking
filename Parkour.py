import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage
import math
import matplotlib.image as mpimg
import scipy.misc
from scipy import ndimage
class Parkour():
    def __init__(self):
        self.image =plt.imread("robot.png")
    
    def drawPlot(self, x, y, theta, targetX=20, targetY=-10, targetT=math.radians(180), show=False):
        xlim = 8
        ylim = 8
        plt.cla()
        
        #print(math.degrees(theta[-1]))
        if(math.degrees(theta[-1])<0):
            plt.imshow((ndimage.rotate(self.image, (270+math.degrees(theta[-1]))) * 255).astype(np.uint8), extent=[x[-1]-1*xlim, x[-1]+1*xlim, y[-1]-0.75*ylim, y[-1]+0.75*ylim], zorder=2)
            #plt.imshow((ndimage.rotate(self.image, (-90+math.degrees(targetT))) * 255).astype(np.uint8), extent=[targetX-1*xlim, targetX+1*xlim, targetY-0.75*ylim, targetY+0.75*ylim], zorder=2)
        else:
            plt.imshow((ndimage.rotate(self.image, (-90+math.degrees(theta[-1]))) * 255).astype(np.uint8), extent=[x[-1]-1*xlim, x[-1]+1*xlim, y[-1]-0.75*ylim, y[-1]+0.75*ylim], zorder=2)
            #plt.imshow((ndimage.rotate(self.image, (-90+math.degrees(targetT))) * 255).astype(np.uint8), extent=[targetX-1*xlim, targetX+1*xlim, targetY-0.75*ylim, targetY+0.75*ylim], zorder=2)
        
        #plt.axis("equal")
        plt.grid(True)
        #plt.xlim(-25, 25)
        #plt.ylim(-25, 25)
        
        #plt.xlim(0, 100*m)
        #plt.ylim(0, 50*m)
         
        if show:
            im = mpimg.imread("map.png")
            rotated_img = ndimage.rotate(im, -180)
            plt.imshow(np.flipud(plt.imread('map.png')), origin='lower')
            #plt.imshow(rotated_img)
            plt.plot(x, y, "ob", label="trajectory", zorder=1)

            plt.show()

        else:
            plt.pause(0.0001)
        
        

        

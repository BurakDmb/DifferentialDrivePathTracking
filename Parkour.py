import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage


class Parkour():
    def __init__(self):
        self.image = plt.imread("robot.png")

    def drawPlot(
            self, x, y, theta, targetX=20, targetY=-10,
            targetT=np.radians(180), show=False):
        xlim = 8
        ylim = 8
        plt.cla()
        if(np.degrees(theta[-1]) < 0):
            plt.imshow(
                (ndimage.rotate(self.image, (270+np.degrees(theta[-1]))) * 255)
                .astype(np.uint8), extent=[
                    x[-1]-1*xlim,
                    x[-1]+1*xlim,
                    y[-1]-0.75*ylim,
                    y[-1]+0.75*ylim], zorder=2)

            plt.imshow(
                (ndimage.rotate(self.image, (-90+np.degrees(targetT))) * 255)
                .astype(np.uint8), extent=[
                    targetX-1*xlim,
                    targetX+1*xlim,
                    targetY-0.75*ylim,
                    targetY+0.75*ylim], zorder=2)
        else:
            plt.imshow(
                (ndimage.rotate(self.image, (-90+np.degrees(theta[-1]))) * 255)
                .astype(np.uint8), extent=[
                    x[-1]-1*xlim,
                    x[-1]+1*xlim,
                    y[-1]-0.75*ylim,
                    y[-1]+0.75*ylim], zorder=2)

            plt.imshow(
                (ndimage.rotate(self.image, (-90+np.degrees(targetT))) * 255)
                .astype(np.uint8), extent=[
                    targetX-1*xlim,
                    targetX+1*xlim,
                    targetY-0.75*ylim,
                    targetY+0.75*ylim], zorder=2)

        # plt.axis("equal")
        plt.grid(True)
        plt.xlim(-25, 25)
        plt.ylim(-25, 25)

        if show:
            plt.plot(x, y, "ob", label="trajectory", zorder=1)
            plt.show()
        else:
            plt.pause(0.0001)

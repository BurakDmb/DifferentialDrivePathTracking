import math

import matplotlib.pyplot as plt
def get_points_from_map():
    class AnnoteFinder(object):
        
        """callback for matplotlib to display an annotation when points are
        clicked on.  The point which is closest to the click and within
        xtol and ytol is identified.
        
        Register this function like this:
        
        scatter(xdata, ydata)
        af = AnnoteFinder(xdata, ydata, annotes)
        connect('button_press_event', af)
        """

        def __init__(self, xdata, ydata, annotes, ax=None, xtol=None, ytol=None):
            self.data = list(zip(xdata, ydata, annotes))
            if xtol is None:
                xtol = ((max(xdata) - min(xdata))/float(len(xdata)))/2
            if ytol is None:
                ytol = ((max(ydata) - min(ydata))/float(len(ydata)))/2
            self.xtol = xtol
            self.ytol = ytol
            if ax is None:
                self.ax = plt.gca()
            else:
                self.ax = ax
            self.drawnAnnotations = {}
            self.links = []
            self.points=[]

        def distance(self, x1, x2, y1, y2):
            """
            return the distance between two points
            """
            return(math.sqrt((x1 - x2)**2 + (y1 - y2)**2))

        def __call__(self, event):

            if event.inaxes:

                clickX = event.xdata
                clickY = event.ydata
                print(f"{clickX},{clickY}")
                self.ax.plot(clickX,clickY,'-ro')
                
                self.points.append([clickX,clickY])
                if (self.ax is None) or (self.ax is event.inaxes):
                    annotes = []
                    # print(event.xdata, event.ydata)
                    for x, y, a in self.data:
                        # print(x, y, a)
                        if ((clickX-self.xtol < x < clickX+self.xtol) and
                                (clickY-self.ytol < y < clickY+self.ytol)):
                            annotes.append(
                                (self.distance(x, clickX, y, clickY), x, y, a))
                    if annotes:
                        annotes.sort()
                        distance, x, y, annote = annotes[0]
                        self.drawAnnote(event.inaxes, x, y, annote)
                        for l in self.links:
                            l.drawSpecificAnnote(annote)

        def drawAnnote(self, ax, x, y, annote):
            """
            Draw the annotation on the plot
            """
            if (x, y) in self.drawnAnnotations:
                markers = self.drawnAnnotations[(x, y)]
                for m in markers:
                    m.set_visible(not m.get_visible())
                self.ax.figure.canvas.draw_idle()
            else:
                t = ax.text(x, y, " - %s" % (annote),)
                m = ax.scatter([x], [y], marker='d', c='r', zorder=100)
                self.drawnAnnotations[(x, y)] = (t, m)
                self.ax.figure.canvas.draw_idle()

        def drawSpecificAnnote(self, annote):
            annotesToDraw = [(x, y, a) for x, y, a in self.data if a == annote]
            for x, y, a in annotesToDraw:
                self.drawAnnote(self.ax, x, y, a)
        def get_points(self):
            return self.points
    x = range(10)
    y = range(10)
    annotes = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']
    fig, ax = plt.subplots()
    #ax.scatter(x,y)
    
    af =  AnnoteFinder(x,y, annotes, ax=ax)
    fig.canvas.mpl_connect('button_press_event', af)

    import numpy as np
    import matplotlib.image as mpimg
    import scipy.misc
    from scipy import ndimage
    im = mpimg.imread("map.png")
    rotated_img = ndimage.rotate(im, -180)
    plt.imshow(np.flipud(plt.imread('map.png')), origin='lower')


    plt.show()
    points=af.get_points()
    return points

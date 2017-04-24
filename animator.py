#!/usr/bin/python

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3
from horntorus import *

fig = plt.figure()
ax = p3.Axes3D(fig)

def init():
    ax.scatter([],[],zs=[])
    return ax

def animate(i):
    a = NDHornTorus(2)
    a.setSpeeds([1,2])
    coords = a.timestepGetCart(0.05)
    ax.scatter([coords[0]],[coords[1]],zs=[coords[2]])

anim =\
animation.FuncAnimation(fig,animate,init_func=init,frames=100,interval=20,blit=True)
anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

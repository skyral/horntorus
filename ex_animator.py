"""
============
3D animation
============

A simple example of an animated plot... In 3D!
"""
from primes import *
from random import random
from math import floor,pi,e
timestep = .002 #timestep in natural unit of time
phasestep = .01
numphases = int(1./phasestep)
numts = 500 #number of timesteps
numTori = 6
tdim = 8
#thMt = [1.,1.] + [0.]*(tdim-2)
#thMs = [2.] + [1.] + [0.]*(tdim-3)  + [1.]
#thMs = [0.]*tdim
#thMt = [0.,0.] + [2.]*(tdim-3) + [0.]
thMt = [0.]*tdim
phases = [0]*tdim
thMs = [0.]*(tdim-1) + [1.]
#thMt = [int(round(random())) for i in range(tdim)]
offsets = [0.]*tdim
offsett = [0.]*tdim
#offsets = [6,5,4,3,2,1,1,2,3,4,5,6]
#offsett = [1,2,1] + [0.]*(tdim-3)
#offsett = getNRandomPrimes(range(tdim),tdim)
nat = (1,1)
#speeds = [[(2,1),nat,(1,2)],[(1,2),nat,(2,1)]]
#speeds = [[nat,nat,nat,nat,nat,nat,nat,(8,1)]]
#speeds = [[(1,1.5),(1,1.5),nat],[(1,1.5),(1,1.5),(2,1)]]
#speeds = [[(7,1),(5,1),(1,3)]]
#speeds = [[(7,1),(1,4),(1,2)]]
#speeds = [[(1,1)]*(tdim-3) + [(1,1),(2,1),(2,1)], [(1,1)]*(tdim-3) + [(1,1),(1,2),(1,2)]]
#numTori = len(speeds)
#offset = [int(round(random()))]*tdim
#thM = [3.,2.,1.,0.,0.,1.,2.,3.]
#thM = [0.]*tdim
#thM = getPrimes(tdim)
#offset = [0.,0.,0.,0.,0.,0.,0.,0.]
#thM = [0.,0.,1.]
#offset = [0.,2.,0.]
#offset = getNRandomPrimes(range(tdim),tdim)
#offset = [o-1 for o in offset]
#thM = [1.,0.]
#offset = [0.,1.]

#proj = [[1,1] + [0]*(tdim-2)  + [1]]*numTori
#proj = [[1,1,1] + [0]*(tdim-3)]*numTori
proj = [[1,1,1,0,0,0,0,0,0]]*numTori
section = [None,None,None,0.,0.,0.,0.]
useproj = True
#useproj = False
usesection = False
#usesection = True


#primes = getNRandomPrimes(range(6),numTori/2)
primes = getNRandomPrimes(range(numTori),numTori)
startind = int(floor(random()*(len(primes)-numTori)))
#speeds = [[1/(1+offset1+th1M*i),1/(1+offset2+th2M*i)] for i in range(numTori/2)]
#speeds += [[(1+offset1+th1M*i),(1+offset2+th2M*i)] for i in range(numTori/2)]
#speeds = [[(1+offset1+th1M*i),(1+offset2+th2M*i)] for i in range(numTori)]
#print speeds
if 'speeds' not in locals():
    speeds = [[(1+offsets[j]+thMs[j]*i, 1+offsett[j]+thMt[j]*i) for j in range(tdim)] for i in range(numTori)]
tols = [[(1/(2*pi))*timestep*(s/t) for s,t in sp] + [(1/(2*pi))*timestep] for sp in speeds]
#speeds = [[float(p),1.] for p in primes]
#speeds += [[1.,float(p)] for p in primes]
#speeds = [[float(p),1.] for p in primes]
#speeds = [[1/2.,1],[1/3.,1],[1/4.,1],[2,1],[3,1],[4,1]]

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from horntorus import *

def genTorusPath(t,ts,nts):
    #expects 2D horn torus
    path = np.empty((3,nts))
    for i in range(nts):
        x,y,z = t.timestepGetCart(ts)
        path[0][i] = x
        path[1][i] = y
        path[2][i] = z
    return path

def genNDTorusSectionPath(ndt,sec,tol,ts,nts):
    path = np.empty((3,nts))
    for i in range(nts):
        arr = ndt.timestepGetCart(ts,'section',sec,tol)
        if(not arr):
            path[0][i] = 0
            path[1][i] = 0
            path[2][i] = 0
        else:
            path[0][i] = arr[0]
            path[1][i] = arr[1]
            path[2][i] = arr[2]
    return path

def genNDTorusProjPath(ndt,proj,ts,nts):
    ndim = 0
    for p in proj:
        if(p is not 0):
            ndim += 1
    path = np.empty((ndim,nts))
    for i in range(nts):
        coords = ndt.timestepGetCart(ts,'proj',proj)
        for j in range(ndim):
            path[j][i] = coords[j]
    return path


def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        '''
        if(num > 4):
            line.set_data(data[0:2, num-4:num])
            line.set_3d_properties(data[2, num-4:num])
        else:
            line.set_data(data[0:2, :num])
            line.set_3d_properties(data[2, :num])
        '''
        line.set_data(data[0:2, :num])
        line.set_3d_properties(data[2, :num])
    return lines

def next_line(num, dataLines, lines):
    for line, data in zip(lines, dataLines[num]):
        # NOTE: there is no .set_data() for 3 dim data...
        '''
        if(num > 4):
            line.set_data(data[0:2, num-4:num])
            line.set_3d_properties(data[2, num-4:num])
        else:
            line.set_data(data[0:2, :num])
            line.set_3d_properties(data[2, :num])
        '''
        line.set_data(data[0:2, :])
        line.set_3d_properties(data[2, :])
    return lines

def next_pcollection(num, dataPts, pcolls):
    for pcoll, data in zip(pcolls, dataPts[num]):
        import ipdb; ipdb.set_trace()
        pcoll._offsets3d(data[0:3,:])
        pcoll.set_color(data[3,:])
    return pcolls

def minandmax(data):
    '''returns minimum and maximum along each axis for
    list of arrays'''
    xmax = 0
    xmin = 2e32
    ymax = 0
    ymin = 2e32
    zmax = 0
    zmin = 2e32
    for arr in data:
        maxes = np.amax(arr,1)
        if(maxes[0] > xmax):
            xmax = maxes[0]
        if(maxes[1] > ymax):
            ymax = maxes[1]
        if(maxes[2] > zmax):
            zmax = maxes[2]
        mins = np.amin(arr,1)
        if(mins[0] < xmin):
            xmin = mins[0]
        if(mins[1] < ymin):
            ymin = mins[1]
        if(mins[2] < zmin):
            zmin = mins[2]
    return ([xmin,xmax],[ymin,ymax],[zmin,zmax])

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# Fifty lines of random 3-D lines
#data = [Gen_RandLine(25, 3) for index in range(50)]

#first generate the tori
toruses = [NDHornTorus(tdim,phases=phases) for i in range(numTori)]
#toruses = [HornTorus() for i in range(1)]
#then set the speeds for each torus
#speeds = [[th1M*i+1.,th2M*i+1.] for i in range(numTori)]
for i,torus in enumerate(toruses):
    torus.setSpeeds(speeds[i])
#now generate their paths to be animated
#data = [genTorusPath(torus,timestep,numts) for torus in toruses]

for step in range(numphases):
    newphases = [ph + step*phasestep*2.*pi for ph in phases]
    for i, torus in enumerate(toruses):
        torus.setAngles(newphases)
    if(useproj):
        if 'data' in locals():
            data += [[genNDTorusProjPath(torus,proj[i],timestep,numts) for i, torus in
                enumerate(toruses)]]
        else:
            data = [[genNDTorusProjPath(torus,proj[i],timestep,numts) for i, torus in
                enumerate(toruses)]]
    elif(usesection):
        if 'data' in locals():
            data += [[genNDTorusSectionPath(torus,section,tols[i],timestep,numts) for i, torus in
                enumerate(toruses)]]
        else:
            data = [[genNDTorusSectionPath(torus,section,tols[i],timestep,numts) for i, torus in
                enumerate(toruses)]]
    else:
        if 'data' in locals():
            data += [[genTorusPath(torus,timestep,numts) for torus in toruses]]
        else:
            data = [[genTorusPath(torus,timestep,numts) for torus in toruses]]
# Creating fifty line objects.
# NOTE: Can't pass empty arrays into 3d version of plot()
needs_color = False
if(len(data[0][0]) > 3):
    needs_color = True
    maximum = np.amax(data)
    minimum = np.amin(data)
    lines = [ax.scatter3D(dat[0, :], dat[1, :], zs=dat[2,\
        :],c=dat[3,:],vmin=minimum,vmax=maximum) for dat in data[0]]
else:
    lines = [ax.plot(dat[0, :], dat[1, :], dat[2,:])[0] for dat in data[0]]

# Setting the axes properties
'''
xlim,ylim,zlim = minandmax(data[0])
if(xlim[0]*-1 > xlim[1]):
    xlim = [xlim[0],-xlim[0]]
else:
    xlim = [-xlim[1],xlim[1]]
if(ylim[0]*-1 > ylim[1]):
    ylim = [ylim[0],-ylim[0]]
else:
    ylim = [-ylim[1],ylim[1]]
if(zlim[0]*-1 > zlim[1]):
    zlim = [zlim[0],-zlim[0]]
else:
    zlim = [-zlim[1],zlim[1]]
coord = max(zlim[1],xlim[1],ylim[1])
coords = [-coord,coord]
'''
maximum = np.amax(data)
minimum = np.amin(data)
if(-minimum > maximum):
    coords = [minimum,-minimum]
else:
    coords = [-maximum,maximum]

#ax.set_xlim3d([data,1/pi])
#ax.set_xlim3d(xlim)
ax.set_xlim3d(coords)
ax.set_xlabel('X')

#ax.set_ylim3d([-1/pi,1/pi])
#ax.set_ylim3d(ylim)
ax.set_ylim3d(coords)
ax.set_ylabel('Y')

#ax.set_zlim3d([-1/(2*pi), 1/(2*pi)])
#ax.set_zlim3d([-1/(pi), 1/(pi)])
#ax.set_zlim3d(zlim)
ax.set_zlim3d(coords)
ax.set_zlabel('Z')

ax.set_title('3D Test')
ax.legend

# Creating the Animation object
#line_ani = animation.FuncAnimation(fig, update_lines, numts, fargs=(data[0], lines),
#                                   interval=30, blit=False)
if(needs_color):
    line_ani = animation.FuncAnimation(fig, next_pcollection, numphases, fargs=(data, lines),
                                   interval=10, blit=False)
else:
    line_ani = animation.FuncAnimation(fig, next_line, numphases, fargs=(data, lines),
                                   interval=10, blit=False)

#line_ani.save('horntori_{0}_{1}s_{2}t'.format(numTori,thMs,thMt),fps=30,extra_args=['-vcodec','libx264'])
plt.show()

from math import pi, cos, sin, pow

class Circle:
    def __init__(self,r=1./(2.*pi)):
        '''Sets the radius of the circle'''
        self.r = float(r)

    def setAngle(self,u):
        '''sets angle u and cartesian coordinates (x,y)'''
        self.u = float(u)
        self.setCartCoord()

    def setCartCoord(self):
        self.x = self.r*cos(self.u)
        self.y = self.r*sin(self.u)

    def getCartCoord(self):
        return (self.x,self.y)

    def stepAngle(self,du):
        self.u = self.u + float(du)
        self.u = self.u % 2*pi

class HornTorus:
    def __init__(self,r=1./(2*pi)):
        '''Sets the radius of the horn torus'''
        self.r = float(r)
        self.speeds = [0.,0.]
        self.angles = [pi]*2
        self.setCartCoords()

    def setAngles(self,angles):
        '''sets the two angles parameterizing the location
        on the surface of the torus, also sets the x,y,z location'''
        self.angles  = angles
        self.setCartCoords()

    def setCartCoords(self):
        self.x = (self.r + self.r*cos(self.angles[0]))*cos(self.angles[1])
        self.y = (self.r + self.r*cos(self.angles[0]))*sin(self.angles[1])
        self.z = self.r*sin(self.angles[0])

    def getCartCoords(self):
        '''gets the cartesian coordinates for given angles nu and u'''
        return (self.x,self.y,self.z)

    def setSpeeds(self,sl=0):
        #speeds are in revolutions per 1 unit of natural time
        if(sl == 0):
            self.s = [1]*2.
            self.t = [1]*2.
            self.speeds = [2*pi]*2
        else:
            self.s = [sli[0] for sli in sl]
            self.t = [sli[1] for sli in sl]
            self.speeds = [float(s)*2*pi/t[i] for i, s in enumerate(self.s)]

    def stepAngles(self,dt):
        self.angles = [th + dt*self.speeds[i] for i, th in enumerate(self.angles)]

    def timestepGetCart(self,dt):
        self.stepAngles(dt)
        self.setCartCoords()
        return self.cart_coords


class NDHornTorus:
    def __init__(self,ndim,r=1./(pi*2.),phases=1):
        '''sets the radius and number of dimensions'''
        self.r = float(r)
        self.ndim = ndim
        if(phases == 1):
            self.phases = [pi]*self.ndim
        else:
            self.phases = phases
        self.angles = self.phases
        self.cart_coords = [0.]*(self.ndim+1)
        self.setCartCoords()
        self.speeds = [0.]*self.ndim

    def setAngles(self,anglist):
        '''sets the angles parameterizing the location on the
        n dimensional torus'''
        self.angles = anglist

    def setSpeeds(self,sl=0):
        #speeds are in revolutions per 1 unit of natural time
        if(sl == 0):
            self.s = [1]*self.ndim
            self.t = [1]*self.ndim
        else:
            self.s = [sli[0] for sli in sl]
            self.t = [sli[1] for sli in sl]
            strp = ''
            for i in range(len(self.s)):
                strp += '{0}/{1} '.format(self.s[i],self.t[i])
            print strp
            self.speeds = [s*2.*pi/self.t[i] for i, s in enumerate(self.s)]

    def setCartCoords(self):
        '''sets the cartesian coordinates from the parameterized
        coordinates'''
        x = self.r
        for n in range(self.ndim):
            self.cart_coords[self.ndim-n] = sin(self.angles[n])*x
            x = x*cos(self.angles[n]) + self.r
        self.cart_coords[0] = x - self.r

    def stepAngles(self,dt):
        self.angles = [th + dt*self.speeds[i] for i, th in enumerate(self.angles)]

    def timestepGetCart(self,dt,proj=None,points=None,tol=None):
        self.stepAngles(dt)
        self.setCartCoords()
        if(not proj):
            return self.cart_coords
        elif(proj == 'proj'):
            return self.getCartProjection(points)
        elif(proj == 'section'):
            return self.getCartCrossSection(points,tol)

    def getNVol(self,n):
        return pow((2*pi*self.r),n)

    def getCartProjection(self,proj):
        '''proj is array of 1 or 0 of length n+1,
        0 meaning project onto that coordinate'''
        newarr = []
        for i, p in enumerate(proj):
            if(p == 0):
                continue
            else:
                newarr.append(self.cart_coords[i])
        return newarr

    def getCartCrossSection(self,section,tol):
        '''section is array of floats, none if
        that dimension is allowed over the range
        tol is array of floats corresponding to allowed
        tolerance for that coordinate. returns null if 
        point is not within tolerance'''
        new_coords = []
        print 'section is {0}'.format(section)
        print 'tol is {0}'.format(tol)
        for i,coord in enumerate(self.cart_coords):
            if(section[i] == None):
                new_coords.append(self.cart_coords[i])
            else:
                if(abs(self.cart_coords[i]-section[i]) < tol[i]):
                    continue
                else:
                    return None
        return new_coords

    def getDistance(self,p1,p2):
        '''expects p1 and p2 n dimensional lists of angles
        or <n dimensions (p1 and p2 must be equal). in this case it will project onto those
        angles setting the remaining angles to some function of the varying angles. finds the
        shortest distance between the two points given.'''
        #string for referencing coordinates instead of numbers
        return 

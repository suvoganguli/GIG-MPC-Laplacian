import numpy as np
from utils import *
def pathInfo(startPoint, endPoint, pathWidth, case):

    if case == 'roadStraightNorth':
        n = 100

        dN = np.array([endPoint[1] - startPoint[1]])/n
        pathE = np.zeros(n) + startPoint[0]
        pathN = np.arange(startPoint[1],endPoint[1],dN)

        pathChi = np.zeros(n)
        for k in range(n-1):
            num = np.array([pathE[k+1] - pathE[k]])
            den = np.array([pathN[k+1] - pathN[k]])
            if den != 0:
                pathChi[k] = np.arctan(num/den)
            else:
                pathChi[k] = np.sign(num)*np.pi/2
        pathChi[n-1] = pathChi[n-2]


    # Generic below this
    dvec1 = np.array([-pathWidth/2,0.0])
    dvec2 = np.array([+pathWidth/2,0.0])

    pathLeftBoundaryE = np.zeros(n)
    pathLeftBoundaryN = np.zeros(n)
    pathRightBoundaryE = np.zeros(n)
    pathRightBoundaryN = np.zeros(n)

    for k in range(n):
        dvec3 = rotate(dvec1,pathChi[k])
        dvec4 = rotate(dvec2,pathChi[k])

        pathLeftBoundaryE[k] = pathE[k] + dvec3[0]
        pathLeftBoundaryN[k] = pathN[k] + dvec3[1]

        pathRightBoundaryE[k] = pathE[k] + dvec4[0]
        pathRightBoundaryN[k] = pathN[k] + dvec4[1]


    class path():
        def __init__(self):
            self.startPoint = startPoint
            self.endPoint = endPoint
            self.pathE = pathE
            self.pathN = pathN
            self.pathChi = pathChi
            self.pathLeftBoundaryE = pathLeftBoundaryE
            self.pathLeftBoundaryN = pathLeftBoundaryN
            self.pathRightBoundaryE = pathRightBoundaryE
            self.pathRightBoundaryN = pathRightBoundaryN

            pass

    return path

def pathSegments(path):



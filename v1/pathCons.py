import numpy as np

def pathConsData(path):
    npts = len(path.PathCenterEndPointsX)

    consPath_AR = np.zeros([npts,2])
    consPath_bR = np.zeros(npts)

    for k in range(npts-1):
        x1 = path.PathRightEndPointsX[k]
        y1 = path.PathRightEndPointsY[k]
        x2 = path.PathRightEndPointsX[k+1]
        y2 = path.PathRightEndPointsY[k+1]

        consPath_AR[k,0] = (y2 - y1) # LHS
        consPath_AR[k,1] = -(x2 - x1) # LHS
        consPath_bR[k] = x1 * y2 - x2 * y1 # RHS

    consPath_AL = np.zeros([npts,2])
    consPath_bL = np.zeros(npts)

    for k in range(npts - 1):
        x1 = path.PathLeftEndPointsX[k]
        y1 = path.PathLeftEndPointsY[k]
        x2 = path.PathLeftEndPointsX[k + 1]
        y2 = path.PathLeftEndPointsY[k + 1]

        consPath_AL[k,0] = (y2 - y1)  # LHS
        consPath_AL[k,1] = -(x2 - x1)  # LHS
        consPath_bL[k] = x1 * y2 - x2 * y1  # RHS


    class consPath:
        def __init__(self):
            self.AR = consPath_AR
            self.bR = consPath_bR
            self.AL = consPath_AL
            self.bL = consPath_bL
            pass
        
    return consPath


import numpy as np

def pathConsData(path):
    npts = len(path.PathCenterEndPointsE)

    consPath_AR = np.zeros([npts,2])
    consPath_bR = np.zeros(npts)

    for k in range(npts-1):
        E1 = path.PathRightEndPointsE[k]
        N1 = path.PathRightEndPointsN[k]
        E2 = path.PathRightEndPointsE[k+1]
        N2 = path.PathRightEndPointsN[k+1]

        consPath_AR[k,0] = (N2 - N1) # LHS
        consPath_AR[k,1] = -(E2 - E1) # LHS
        consPath_bR[k] = E1 * N2 - E2 * N1 # RHS

    consPath_AL = np.zeros([npts,2])
    consPath_bL = np.zeros(npts)

    for k in range(npts - 1):
        E1 = path.PathLeftEndPointsE[k]
        N1 = path.PathLeftEndPointsN[k]
        E2 = path.PathLeftEndPointsE[k + 1]
        N2 = path.PathLeftEndPointsN[k + 1]

        consPath_AL[k,0] = (N2 - N1)  # LHS
        consPath_AL[k,1] = -(E2 - E1)  # LHS
        consPath_bL[k] = E1 * N2 - E2 * N1  # RHS


    class consPath:
        def __init__(self):
            self.AR = consPath_AR
            self.bR = consPath_bR
            self.AL = consPath_AL
            self.bL = consPath_bL
            pass
        
    return consPath


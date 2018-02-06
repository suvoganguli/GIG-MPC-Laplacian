import numpy as np

def pathCostData(path):
    npts = len(path.PathCenterEndPointsE)

    costAlongPath_A = np.zeros([npts-1,2])
    costAlongPath_b = np.zeros(npts-1)

    for k in range(npts-1):
        x1 = path.PathCenterEndPointsE[k]
        N1 = path.PathCenterEndPointsN[k]
        x2 = path.PathCenterEndPointsE[k+1]
        N2 = path.PathCenterEndPointsN[k+1]

        costAlongPath_A[k,0] = (N2 - N1) # LHS
        costAlongPath_A[k,1] = -(x2 - x1) # LHS
        costAlongPath_b[k] = x1 * N2 - x2 * N1 # RHS

    costAcrossPath_A = np.zeros([npts,2])
    costAcrossPath_b = np.zeros(npts)

    for k in range(npts):

        E1 = path.PathRightEndPointsE[k]
        N1 = path.PathRightEndPointsN[k]
        E2 = path.PathLeftEndPointsE[k]
        N2 = path.PathLeftEndPointsN[k]

        costAcrossPath_A[k,0] = (N2 - N1)  # LHS
        costAcrossPath_A[k,1] = -(E2 - E1)  # LHS
        costAcrossPath_b[k] = E1 * N2 - E2 * N1  # RHS


    class costAlongPath:
        def __init__(self):
            self.A = costAlongPath_A
            self.b = costAlongPath_b
            pass

    class costAcrossPath:
        def __init__(self):
            self.A = costAcrossPath_A
            self.b = costAcrossPath_b
            pass

    return costAlongPath, costAcrossPath
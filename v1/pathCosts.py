import numpy as np

def pathCostData(path):
    npts = len(path.PathCenterEndPointsX)

    costAlongPath_A = np.zeros([npts-1,2])
    costAlongPath_b = np.zeros(npts-1)

    for k in range(npts-1):
        x1 = path.PathCenterEndPointsX[k]
        y1 = path.PathCenterEndPointsY[k]
        x2 = path.PathCenterEndPointsX[k+1]
        y2 = path.PathCenterEndPointsY[k+1]

        costAlongPath_A[k,0] = (y2 - y1) # LHS
        costAlongPath_A[k,1] = -(x2 - x1) # LHS
        costAlongPath_b[k] = x1 * y2 - x2 * y1 # RHS

    costAcrossPath_A = np.zeros([npts,2])
    costAcrossPath_b = np.zeros(npts)

    for k in range(npts):

        x1 = path.PathRightEndPointsX[k]
        y1 = path.PathRightEndPointsY[k]
        x2 = path.PathLeftEndPointsX[k]
        y2 = path.PathLeftEndPointsY[k]

        costAcrossPath_A[k,0] = (y2 - y1)  # LHS
        costAcrossPath_A[k,1] = -(x2 - x1)  # LHS
        costAcrossPath_b[k] = x1 * y2 - x2 * y1  # RHS


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
import numpy as np

def roadCostData(lane1, lane2):
    npts = len(lane1.LaneCenterEndPointsX)

    costLane1_A = np.zeros([npts-1,2])
    costLane1_b = np.zeros(npts-1)

    for k in range(npts-1):
        x1 = lane1.LaneCenterEndPointsX[k]
        y1 = lane1.LaneCenterEndPointsY[k]
        x2 = lane1.LaneCenterEndPointsX[k+1]
        y2 = lane1.LaneCenterEndPointsY[k+1]

        costLane1_A[k,0] = (y2 - y1) # LHS
        costLane1_A[k,1] = -(x2 - x1) # LHS
        costLane1_b[k] = x1 * y2 - x2 * y1 # RHS

    costLane2_A = np.zeros([npts-1, 2])
    costLane2_b = np.zeros(npts-1)

    for k in range(npts - 1):
        x1 = lane2.LaneCenterEndPointsX[k]
        y1 = lane2.LaneCenterEndPointsY[k]
        x2 = lane2.LaneCenterEndPointsX[k + 1]
        y2 = lane2.LaneCenterEndPointsY[k + 1]

        costLane2_A[k,0] = (y2 - y1)  # LHS
        costLane2_A[k,1] = -(x2 - x1)  # LHS
        costLane2_b[k,] = x1 * y2 - x2 * y1  # RHS


    costAcross_A = np.zeros([npts,2])
    costAcross_b = np.zeros(npts)

    for k in range(npts):

        x1 = lane1.LaneRightEndPointsX[k]
        y1 = lane1.LaneRightEndPointsY[k]
        x2 = lane2.LaneLeftEndPointsX[k]
        y2 = lane2.LaneLeftEndPointsY[k]

        costAcross_A[k,0] = (y2 - y1)  # LHS
        costAcross_A[k,1] = -(x2 - x1)  # LHS
        costAcross_b[k] = x1 * y2 - x2 * y1  # RHS


    class costLane1:
        def __init__(self):
            self.costLane_A = costLane1_A
            self.costLane_b = costLane1_b
            pass

    class costLane2:
        def __init__(self):
            self.costLane_A = costLane2_A
            self.costLane_b = costLane2_b
            pass

    class costAcross:
        def __init__(self):
            self.costAcross_A = costAcross_A
            self.costAcross_b = costAcross_b
            pass

    return costLane1, costLane2, costAcross
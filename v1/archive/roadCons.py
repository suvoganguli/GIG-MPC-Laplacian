import numpy as np

def roadConsData(lane1, lane2):
    npts = len(lane1.LaneCenterEndPointsX)

    consLane1_AR = np.zeros([npts,2])
    consLane1_bR = np.zeros(npts)

    for k in range(npts-1):
        x1 = lane1.LaneRightEndPointsX[k]
        y1 = lane1.LaneRightEndPointsY[k]
        x2 = lane1.LaneRightEndPointsX[k+1]
        y2 = lane1.LaneRightEndPointsY[k+1]

        consLane1_AR[k,0] = (y2 - y1) # LHS
        consLane1_AR[k,1] = -(x2 - x1) # LHS
        consLane1_bR[k] = x1 * y2 - x2 * y1 # RHS

    consLane1_AL = np.zeros([npts,2])
    consLane1_bL = np.zeros(npts)

    for k in range(npts - 1):
        x1 = lane1.LaneLeftEndPointsX[k]
        y1 = lane1.LaneLeftEndPointsY[k]
        x2 = lane1.LaneLeftEndPointsX[k + 1]
        y2 = lane1.LaneLeftEndPointsY[k + 1]

        consLane1_AL[k,0] = (y2 - y1)  # LHS
        consLane1_AL[k,1] = -(x2 - x1)  # LHS
        consLane1_bL[k] = x1 * y2 - x2 * y1  # RHS


    consLane2_AR = np.zeros([npts,2])
    consLane2_bR = np.zeros(npts)

    for k in range(npts-1):
        x1 = lane2.LaneRightEndPointsX[k]
        y1 = lane2.LaneRightEndPointsY[k]
        x2 = lane2.LaneRightEndPointsX[k+1]
        y2 = lane2.LaneRightEndPointsY[k+1]

        consLane2_AR[k,0] = (y2 - y1) # LHS
        consLane2_AR[k,1] = -(x2 - x1) # LHS
        consLane2_bR[k] = x1 * y2 - x2 * y1 # RHS

    consLane2_AL = np.zeros([npts,2])
    consLane2_bL = np.zeros(npts)

    for k in range(npts - 1):
        x1 = lane2.LaneLeftEndPointsX[k]
        y1 = lane2.LaneLeftEndPointsY[k]
        x2 = lane2.LaneLeftEndPointsX[k + 1]
        y2 = lane2.LaneLeftEndPointsY[k + 1]
        consLane2_AL[k,0] = (y2 - y1)  # LHS
        consLane2_AL[k,1] = -(x2 - x1)  # LHS
        consLane2_bL[k] = x1 * y2 - x2 * y1  # RHS


    class consLane1:
        def __init__(self):
            self.consLane_AR = consLane1_AR
            self.consLane_bR = consLane1_bR
            self.consLane_AL = consLane1_AL
            self.consLane_bL = consLane1_bL
            pass

    class consLane2:
        def __init__(self):
            self.consLane_AR = consLane2_AR
            self.consLane_bR = consLane2_bR
            self.consLane_AL = consLane2_AL
            self.consLane_bL = consLane2_bL
            pass

    return consLane1, consLane2


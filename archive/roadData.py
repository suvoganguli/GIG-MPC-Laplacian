import numpy as np
import matplotlib.pyplot as plt


def roadInputData(case):

    if case == 'roadStraightNorth':
        nRoadSegments = 50
        roadSegmentLengths = 10*np.ones(nRoadSegments) # ft
        roadSegmentCurvatures = np.zeros(nRoadSegments) # ft
        laneWidth = 10.0 # ft
        data = {'case':case,
                'nRoadSegments': nRoadSegments,
                'laneWidth': laneWidth,
                'numberOfLanes': 2,
                'roadStart': [0.0, 0.0, np.pi/2], # ft, ft, rad (road theta is w.r.t +x axis)
                'roadSegmentLengths': roadSegmentLengths,
                'roadSegmentCurvatures': roadSegmentCurvatures,
                }

    elif case == 'roadStraightEast':
        nRoadSegments = 50
        roadSegmentLengths = 10 * np.ones(nRoadSegments)  # ft
        roadSegmentCurvatures = np.zeros(nRoadSegments) # ft
        laneWidth = 10.0  # ft
        data = {'case': case,
                'nRoadSegments': nRoadSegments,
                'laneWidth': laneWidth,
                'numberOfLanes': 2,
                'roadStart': [0.0, 0.0, 0.0], # ft, ft, rad
                'roadSegmentLengths': roadSegmentLengths,
                'roadSegmentCurvatures': roadSegmentCurvatures,
                }

    elif case == 'roadStraightNorthEast':
        nRoadSegments = 50
        roadSegmentLengths = 10 * np.ones(nRoadSegments)  # ft
        roadSegmentCurvatures = np.zeros(nRoadSegments) # ft
        laneWidth = 10.0  # ft
        data = {'case': case,
                'nRoadSegments': nRoadSegments,
                'laneWidth': laneWidth,
                'numberOfLanes': 2,
                'roadStart': [0.0, 0.0, 45.0*np.pi/180], # ft, ft, rad
                'roadSegmentLengths': roadSegmentLengths,
                'roadSegmentCurvatures': roadSegmentCurvatures,
                }


    elif case == 'UGVDemo1':

        track_length = 360.0 # ft
        track_diameter = 78.0 # ft
        section_length = 10.0 # ft

        L = track_length     # ft
        D = track_diameter  # ft
        W = section_length  # ft

        R = D/2
        C = 1/R

        n1 = int(np.floor(L / W))
        L1 = np.ones(n1) * W
        dL1 = L - n1 * W

        theta = 2*np.arcsin(W / (2 * R))
        n2 = int(np.floor(np.pi / theta))
        L2 = np.ones(n2) * W
        dtheta = np.pi - n2 * theta
        dL2 = 2 * R * np.sin(dtheta / 2)

        # start point of the segment [X, Y, theta]
        ROAD_START = [0.0, 0.0, np.pi / 2]

        L1 = L1.tolist()
        dL1 = [dL1]
        L2 = L2.tolist()
        dL2 = [dL2]

        # Length of the road segment [ft]
        ROAD_LENGTH = L1 + dL1 + L2 + dL2 + L1 + dL1 + L2 + dL2

        #print(len(L1))
        #print(len(dL1))
        #print(len(L2))
        #print(len(dL2))
        #print(len(ROAD_LENGTH))

        # Curvature of the road  [rad / ft]

        C1 = [0]*n1
        C2 = [0]
        C3 = [C]*n2
        C4 = [C]
        C5 = [0]*n1
        C6 = [0]
        C7 = [C]*n2
        C8 = [C]

        ROAD_CURVATURE = C1 + C2 + C3 + C4 + C5 + C6 + C7 + C8

        nRoadSegments = len(ROAD_LENGTH)
        roadSegmentLengths = ROAD_LENGTH # ft
        roadSegmentCurvatures = ROAD_CURVATURE # ft
        laneWidth = 15.0 # ft

        data = {'case': case,
                'nRoadSegments': nRoadSegments,
                'laneWidth': laneWidth,
                'numberOfLanes': 2,
                'roadStart': [0.0, 0.0, np.pi/2], # ft, ft, rad (road theta is w.r.t +x axis)
                'roadSegmentLengths': roadSegmentLengths,
                'roadSegmentCurvatures': roadSegmentCurvatures,
                }

    elif case == 'UGVDemo1Debug':

        track_length = 360.0 # ft
        track_diameter = 78.0 # ft
        section_length = 10.0 # ft

        L = track_length     # ft
        D = track_diameter  # ft
        W = section_length  # ft

        R = D/2
        C = 1/R

        n1 = int(np.floor(L / W))
        L1 = np.ones(n1) * W
        dL1 = L - n1 * W

        theta = 2 * np.arcsin(W / (2 * R))
        theta_arc = np.pi/6
        n2 = int(np.floor(theta_arc / theta))
        L2 = np.ones(n2) * W
        dtheta = theta_arc - n2 * theta  #np.pi - n2 * theta
        dL2 = 2 * R * np.sin(dtheta / 2)

        # start point of the segment [X, Y, theta]
        ROAD_START = [0.0, 0.0, np.pi / 2]

        L1 = L1.tolist()
        dL1 = [dL1]
        L2 = L2.tolist()
        dL2 = [dL2]

        # Length of the road segment [ft]
        ROAD_LENGTH = L1 + dL1 + L2 + dL2 + L1
        # Curvature of the road  [rad / ft]

        C1 = [0]*n1
        C2 = [0]
        C3 = [C]*n2
        C4 = [C]
        C5 = [0]*n1

        ROAD_CURVATURE = C1 + C2 + C3 + C4 + C5

        nRoadSegments = len(ROAD_LENGTH)
        roadSegmentLengths = ROAD_LENGTH # ft
        roadSegmentCurvatures = ROAD_CURVATURE # ft
        laneWidth = 15.0 # ft

        data = {'case': case,
                'nRoadSegments': nRoadSegments,
                'laneWidth': laneWidth,
                'numberOfLanes': 2,
                'roadStart': [0.0, 0.0, np.pi/2], # ft, ft, rad (road theta is w.r.t +x axis)
                'roadSegmentLengths': roadSegmentLengths,
                'roadSegmentCurvatures': roadSegmentCurvatures,
                }

    else:
        nRoadSegments = -1
        roadSegmentLengths = -1
        roadSegmentCurvatures = -1
        laneWidth = -1
        data = {'case': -1,
                'nroadSegments': nRoadSegments,
                'laneWidth': laneWidth,
                'numberOfLanes': -1,
                'roadStart': [-1, -1, -1],
                'roadSegmentLengths': roadSegmentLengths,
                'roadSegmentCurvatures': roadSegmentCurvatures,
                }
    return data



def roadDetailedData(roadInputData):

    # get roadInput dictionary values
    case = roadInputData['case']
    nRoadSegments = roadInputData['nRoadSegments']
    laneWidth = roadInputData['laneWidth']
    numberOfLanes = roadInputData['numberOfLanes']
    roadStart = roadInputData['roadStart']
    roadSegmentLengths = roadInputData['roadSegmentLengths']
    roadSegmentCurvatures = roadInputData['roadSegmentCurvatures']

    #  Set the initial location of the road segment
    x0 = roadStart[0] # ft
    y0 = roadStart[1] # ft
    theta0 = roadStart[2] # rad

    # Waypoint Data
    deltaWP = 1 # ft, Waypoint separation distance

    # LaneWidth/2
    d = laneWidth/2

    # Create zero vectors to hold full road definition
    # X_full = np.empty(0)
    # Y_full = np.empty(0)
    # Theta_full = np.empty(0)
    # LaneLeftX_full = np.empty(0)
    # LaneLeftY_full = np.empty(0)
    # LaneRightX_full = np.empty(0)
    # LaneRightY_full = np.empty(0)
    # X_endpoints = np.empty(0)
    # Y_endpoints = np.empty(0)
    # Theta_endpoints = np.empty(0)

    X_full = np.array([x0])
    Y_full = np.array([y0])
    Theta_full = np.array([theta0])
    LaneLeftX_full = np.array([x0 - d * np.sin(theta0)])
    LaneLeftY_full = np.array([y0 + d * np.cos(theta0)])
    LaneRightX_full = np.array([x0 + d * np.sin(theta0)])
    LaneRightY_full = np.array([y0 - d * np.cos(theta0)])
    X_endpoints = np.array([x0])
    Y_endpoints = np.array([y0])
    Theta_endpoints = np.array([theta0])

    nWP = 0

    # Loop through each set of parameters (each is a road segment)
    #segLengthAll = roadSegmentLengths

    Theta = []

    for i in range(nRoadSegments):

        segCurvature = roadSegmentCurvatures[i]
        segLength = roadSegmentLengths[i]

        n = np.floor(segLength / deltaWP)

        if n != int(n):
            print('segLenth must be an integer multiple of deltaWP')

        road_tht = 2 * np.arcsin((segLength / 2) * segCurvature)

        if n > 0:
            road_dtht = road_tht / n
            Theta = theta0 + road_dtht * (np.arange(n)+1)
        else:
            Theta = np.array([theta0])

        if (Theta[-1] * 180 / np.pi > 269) and (Theta[-1] * 180 / np.pi < 270):

            road_dtht2 = [270 * np.pi / 180 - Theta[-1]]
            Theta[-1] = Theta[-1] + road_dtht2

        nWP = len(Theta)

        X = np.zeros(nWP)
        Y = np.zeros(nWP)
        LaneLeftX = np.zeros(nWP)
        LaneLeftY = np.zeros(nWP)
        LaneRightX = np.zeros(nWP)
        LaneRightY = np.zeros(nWP)

        for j in range(nWP):
            theta = Theta[j]
            x0 = x0 + deltaWP * np.cos(theta)
            y0 = y0 + deltaWP * np.sin(theta)
            X[j] = x0
            Y[j] = y0
            LaneLeftX[j] = x0 - d * np.sin(theta)
            LaneLeftY[j] = y0 + d * np.cos(theta)
            LaneRightX[j] = x0 + d * np.sin(theta)
            LaneRightY[j] = y0 - d * np.cos(theta)

        # Add the segment to the complete road defintion
        last_idx = nWP
        X_full = np.concatenate((X_full, X[0:last_idx]))
        Y_full = np.concatenate((Y_full, Y[0:last_idx]))
        Theta_full = np.concatenate((Theta_full, Theta[0:last_idx]))
        LaneLeftX_full = np.concatenate((LaneLeftX_full, LaneLeftX[0:last_idx]))
        LaneLeftY_full = np.concatenate((LaneLeftY_full, LaneLeftY[0:last_idx]))
        LaneRightX_full = np.concatenate((LaneRightX_full, LaneRightX[0:last_idx]))
        LaneRightY_full = np.concatenate((LaneRightY_full, LaneRightY[0:last_idx]))

        # Reset the starting points for the next segment
        x0 = X[nWP-1]
        y0 = Y[nWP-1]
        theta0 = Theta[nWP-1]

        # Store the endpoints of the segment
        X_endpoints = np.concatenate((X_endpoints, [X[0]]))
        Y_endpoints = np.concatenate((Y_endpoints, [Y[0]]))
        Theta_endpoints = np.concatenate((Theta_endpoints, [Theta[0]]))


    # Lane 1 end points
    lane1LaneCenterEndPointsX = X_endpoints
    lane1LaneCenterEndPointsY = Y_endpoints
    lane1LaneRightEndPointsX = X_endpoints + laneWidth / 2 * np.sin(Theta_endpoints)
    lane1LaneRightEndPointsY = Y_endpoints - laneWidth / 2 * np.cos(Theta_endpoints)
    lane1LaneLeftEndPointsX = X_endpoints - laneWidth / 2 * np.sin(Theta_endpoints)
    lane1LaneLeftEndPointsY = Y_endpoints + laneWidth/2 * np.cos(Theta_endpoints)

    # Lane 2 end points
    lane2LaneCenterEndPointsX = lane1LaneCenterEndPointsX - laneWidth * np.sin(Theta_endpoints)
    lane2LaneCenterEndPointsY = lane1LaneCenterEndPointsY + laneWidth * np.cos(Theta_endpoints)
    lane2LaneRightEndPointsX = lane1LaneLeftEndPointsX
    lane2LaneRightEndPointsY = lane1LaneLeftEndPointsY
    lane2LaneLeftEndPointsX = lane2LaneCenterEndPointsX - laneWidth / 2 * np.sin(Theta_endpoints)
    lane2LaneLeftEndPointsY = lane2LaneCenterEndPointsY + laneWidth/2 * np.cos(Theta_endpoints)

    #print(len(Theta_full))
    #plt.plot(Theta_full*180/np.pi,marker='x')
    #plt.grid(True)
    #plt.show()

    # Save lane data in classes
    class lane1(object):
        def __init__(self):
            self.nRoadSegments = nRoadSegments
            self.roadSegmentLengths = roadSegmentLengths
            self.X = X_full
            self.Y = Y_full
            self.Theta = Theta_full
            self.LaneLeftBoundaryX = LaneLeftX_full
            self.LaneLeftBoundaryY = LaneLeftY_full
            self.LaneRightBoundaryX = LaneRightX_full
            self.LaneRightBoundaryY = LaneRightY_full
            self.LaneCenterEndPointsX = lane1LaneCenterEndPointsX
            self.LaneCenterEndPointsY = lane1LaneCenterEndPointsY
            self.LaneRightEndPointsX = lane1LaneRightEndPointsX
            self.LaneRightEndPointsY = lane1LaneRightEndPointsY
            self.LaneLeftEndPointsX = lane1LaneLeftEndPointsX
            self.LaneLeftEndPointsY = lane1LaneLeftEndPointsY
            self.Theta_endpoints = Theta_endpoints
            pass


    class lane2(object):
        def __init__(self):
            self.nRoadSegments = nRoadSegments
            self.roadSegmentLengths = roadSegmentLengths
            self.X = X_full - laneWidth*np.sin(Theta_full)
            self.Y = Y_full + laneWidth*np.cos(Theta_full)
            self.Theta = Theta_full
            self.LaneLeftBoundaryX = LaneLeftX_full - laneWidth*np.sin(Theta_full)
            self.LaneLeftBoundaryY = LaneLeftY_full + laneWidth*np.cos(Theta_full)
            self.LaneRightBoundaryX = LaneRightX_full - laneWidth*np.sin(Theta_full)
            self.LaneRightBoundaryY = LaneRightY_full + laneWidth*np.cos(Theta_full)
            self.LaneCenterEndPointsX = lane2LaneCenterEndPointsX
            self.LaneCenterEndPointsY = lane2LaneCenterEndPointsY
            self.LaneRightEndPointsX = lane2LaneRightEndPointsX
            self.LaneRightEndPointsY = lane2LaneRightEndPointsY
            self.LaneLeftEndPointsX = lane2LaneLeftEndPointsX
            self.LaneLeftEndPointsY = lane2LaneLeftEndPointsY
            self.Theta_endpoints = Theta_endpoints
            pass


    return lane1, lane2


##################################################
# Tests

if False:
    case = 'UGVDemo1'
    x0, roadData = roadInputData(case)

    lane1Class, lane2Class = roadDetailedData(roadData)
    lane1 = lane1Class()
    lane2 = lane2Class()

    print(roadData['nRoadSegments'])
    print(roadData['laneWidth'])
    print(roadData['numberOfLanes'])
    #print(roadData['roadStart'])
    #print(roadData['roadSegmentLengths'])
    #print(roadData['roadSegmentCurvatures'])
    #print(roadData['obstacle'])

    print(min(lane1.X))
    print(max(lane1.Y))
    #print(lane1.LaneLeftBoundaryX)
    #print(lane1.LaneLeftBoundaryY)
    #print(lane1.LaneRightBoundaryX)
    #print(lane1.LaneRightBoundaryY)
    f, ax = plt.subplots(1, sharex=True)
    ax.plot(lane1.X, lane1.Y, linestyle = '--',color = 'k')
    ax.plot(lane1.LaneLeftBoundaryX, lane1.LaneLeftBoundaryY, linestyle = '-', color = 'k')
    ax.plot(lane1.LaneRightBoundaryX, lane1.LaneRightBoundaryY, linestyle = '-', color = 'k')
    ax.plot(lane2.X, lane2.Y, linestyle = '--',color = 'k')
    ax.plot(lane2.LaneLeftBoundaryX, lane2.LaneLeftBoundaryY, linestyle = '-', color = 'k')
    ax.grid(True)
    plt.show()
    None






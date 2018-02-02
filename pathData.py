import numpy as np

def pathInitData(case, startPoint, endPoint, obstacle = None):

    pathDist = np.linalg.norm(endPoint - startPoint)
    pathSectionLength = 10.0
    nPathSections = int(pathDist/pathSectionLength)
    deltaPathLength = pathDist/pathSectionLength - nPathSections
    pathSectionLengths = pathSectionLength * np.ones(nPathSections)

    if deltaPathLength > 0:
        pathSectionLengths = np.append([pathSectionLengths, deltaPathLength])
        nPathSections = nPathSections + 1

    pathWidth = 10.0
    pathTheta = np.pi/2  # (road theta is w.r.t +East axis)

    pathSectionCurvatures = np.zeros(nPathSections)

    data = {'case':case,
            'pathStart': [startPoint[0], startPoint[1], pathTheta], # ft, ft, rad
            'pathWidth': pathWidth,
            'nPathSections': nPathSections,
            'pathSectionLengths': pathSectionLengths,
            'pathSectionCurvatures': pathSectionCurvatures,
            }

    return data


def pathInputData(case):

    nPathSections = 50
    pathSectionLengths = 10*np.ones(nPathSections) # ft
    pathSectionCurvatures = np.zeros(nPathSections) # ft
    pathWidth = 10.0 # ft
    data = {'case':case,
            'pathStart': [0.0, 0.0, np.pi/2], # ft, ft, rad (road theta is w.r.t +East axis)
            'pathWidth': pathWidth,
            'nPathSections': nPathSections,
            'pathSectionLengths': pathSectionLengths,
            'pathSectionCurvatures': pathSectionCurvatures,
            }

    return data


def pathDetailedData(pathInputData):

    pathStart = pathInputData['pathStart']
    pathWidth = pathInputData['pathWidth']
    nPathSections = pathInputData['nPathSections']
    pathSectionLengths = pathInputData['pathSectionLengths']
    pathSectionCurvatures = pathInputData['pathSectionCurvatures']

    #  Set the initial location of the path section
    x0 = pathStart[0] # ft
    y0 = pathStart[1] # ft
    theta0 = pathStart[2] # rad

    # Waypoint Data
    deltaWP = 1 # ft, Waypoint separation distance

    # pathWidth/2
    d = pathWidth/2

    # Create zero vectors to hold full path definition
    X_full = np.empty(0)
    Y_full = np.empty(0)
    Theta_full = np.empty(0)
    PathLeftX_full = np.empty(0)
    PathLeftY_full = np.empty(0)
    PathRightX_full = np.empty(0)
    PathRightY_full = np.empty(0)
    X_endpoints = np.empty(0)
    Y_endpoints = np.empty(0)
    Theta_endpoints = np.empty(0)

    # Loop through each set of parameters (each is a path section)

    Theta = []

    for i in range(nPathSections):

        secLength = pathSectionLengths[i]
        secCurvature = pathSectionCurvatures[i]

        n = np.floor(secLength / deltaWP)

        if n != int(n):
            print('secLenth must be an integer multiple of deltaWP')

        path_tht = 2 * np.arcsin((secLength / 2) * secCurvature)

        if n > 0:
            path_dtht = path_tht / n
            Theta = theta0 + path_dtht * (np.arange(n+1))
        else:
            Theta = np.array([theta0])

        if (Theta[-1] * 180 / np.pi > 269) and (Theta[-1] * 180 / np.pi < 270):

            path_dtht2 = [270 * np.pi / 180 - Theta[-1]]
            Theta[-1] = Theta[-1] + path_dtht2

        nWP = len(Theta)

        X = np.zeros(nWP)
        Y = np.zeros(nWP)
        PathLeftX = np.zeros(nWP)
        PathLeftY = np.zeros(nWP)
        PathRightX = np.zeros(nWP)
        PathRightY = np.zeros(nWP)

        for j in range(nWP):
            theta = Theta[j]
            X[j] = x0
            Y[j] = y0
            PathLeftX[j] = x0 - d * np.sin(theta)
            PathLeftY[j] = y0 + d * np.cos(theta)
            PathRightX[j] = x0 + d * np.sin(theta)
            PathRightY[j] = y0 - d * np.cos(theta)

            x0 = x0 + deltaWP * np.cos(theta)
            y0 = y0 + deltaWP * np.sin(theta)

        # Add the section to the complete path defintion
        X_full = np.concatenate((X_full, X[:-1]))
        Y_full = np.concatenate((Y_full, Y[:-1]))
        Theta_full = np.concatenate((Theta_full, Theta[:-1]))
        PathLeftX_full = np.concatenate((PathLeftX_full, PathLeftX[:-1]))
        PathLeftY_full = np.concatenate((PathLeftY_full, PathLeftY[:-1]))
        PathRightX_full = np.concatenate((PathRightX_full, PathRightX[:-1]))
        PathRightY_full = np.concatenate((PathRightY_full, PathRightY[:-1]))

        # Reset the starting points for the next section
        x0 = X[nWP-1]
        y0 = Y[nWP-1]
        theta0 = Theta[nWP-1]

        # Store the start points of the section
        X_endpoints = np.concatenate((X_endpoints, [X[0]]))
        Y_endpoints = np.concatenate((Y_endpoints, [Y[0]]))
        Theta_endpoints = np.concatenate((Theta_endpoints, [Theta[0]]))

    X_full = np.concatenate((X_full, X[-1:]))
    Y_full = np.concatenate((Y_full, Y[-1:]))
    Theta_full = np.concatenate((Theta_full, Theta[-1:]))
    PathLeftX_full = np.concatenate((PathLeftX_full, PathLeftX[-1:]))
    PathLeftY_full = np.concatenate((PathLeftY_full, PathLeftY[-1:]))
    PathRightX_full = np.concatenate((PathRightX_full, PathRightX[-1:]))
    PathRightY_full = np.concatenate((PathRightY_full, PathRightY[-1:]))

    # Store the end point of the last section
    X_endpoints = np.concatenate((X_endpoints, [X[-1]]))
    Y_endpoints = np.concatenate((Y_endpoints, [Y[-1]]))
    Theta_endpoints = np.concatenate((Theta_endpoints, [Theta[-1]]))

    # Define final variables for storage in class

    CenterEndPointsX = X_endpoints
    CenterEndPointsY = Y_endpoints
    RightEndPointsX = X_endpoints + pathWidth / 2 * np.sin(Theta_endpoints)
    RightEndPointsY = Y_endpoints - pathWidth / 2 * np.cos(Theta_endpoints)
    LeftEndPointsX = X_endpoints - pathWidth / 2 * np.sin(Theta_endpoints)
    LeftEndPointsY = Y_endpoints + pathWidth/2 * np.cos(Theta_endpoints)


    # Save lane data in classes
    class path(object):
        def __init__(self):
            self.nPathSections = nPathSections
            self.pathSectionLengths = pathSectionLengths
            self.X = X_full
            self.Y = Y_full
            self.Theta = Theta_full
            self.PathLeftBoundaryX = PathLeftX_full
            self.PathLeftBoundaryY = PathLeftY_full
            self.PathRightBoundaryX = PathRightX_full
            self.PathRightBoundaryY = PathRightY_full
            self.PathCenterEndPointsX = CenterEndPointsX
            self.PathCenterEndPointsY = CenterEndPointsY
            self.PathRightEndPointsX = RightEndPointsX
            self.PathRightEndPointsY = RightEndPointsY
            self.PathLeftEndPointsX = LeftEndPointsX
            self.PathLeftEndPointsY = LeftEndPointsY
            self.Theta_endpoints = Theta_endpoints
            pass

    return path
# ----------------------------------------------------------------
# Older version:
#
# def pathInfo(case):
#
#     # --- Define the following ------------------------
#     # - pathStart
#     # - pathEnd
#     # - pathWidth
#     # - pathSectionLength
#     # - pathE
#     # - pathN
#     # - pathChi
#
#     if case == 'pathStraightNorth':
#
#         pathStart = np.array([0,0,0])  # E [ft], N [ft], Chi [rad]
#         pathWidth = 10
#
#         n = 100
#
#         dN = np.array([pathEnd[1] - pathStart[1]])/n
#         pathE = np.zeros(n) + pathStart[0]
#         pathN = np.arange(pathStart[1],pathEnd[1],dN)
#
#         pathSectionLength = dN # [ft]
#
#         pathChi = np.zeros(n)
#         for k in range(n-1):
#             num = np.array([pathE[k+1] - pathE[k]])
#             den = np.array([pathN[k+1] - pathN[k]])
#             if den != 0:
#                 pathChi[k] = np.arctan(num/den)
#             else:
#                 pathChi[k] = np.sign(num)*np.pi/2
#         pathChi[n-1] = pathChi[n-2]
#
#
#     # ----------  Generic below this ---------------------
#
#     dvec1 = np.array([-pathWidth/2,0.0])
#     dvec2 = np.array([+pathWidth/2,0.0])
#
#     pathLeftBoundaryE = np.zeros(n)
#     pathLeftBoundaryN = np.zeros(n)
#     pathRightBoundaryE = np.zeros(n)
#     pathRightBoundaryN = np.zeros(n)
#
#     for k in range(n):
#         dvec3 = rotate(dvec1,pathChi[k])
#         dvec4 = rotate(dvec2,pathChi[k])
#
#         pathLeftBoundaryE[k] = pathE[k] + dvec3[0]
#         pathLeftBoundaryN[k] = pathN[k] + dvec3[1]
#
#         pathRightBoundaryE[k] = pathE[k] + dvec4[0]
#         pathRightBoundaryN[k] = pathN[k] + dvec4[1]
#
#
#     class path():
#         def __init__(self):
#             self.pathStart = pathStart
#             self.pathEnd = pathEnd
#             self.nPathSections = n
#             self.pathWidth = pathWidth
#             self.pathSectionLength = pathSectionLength
#             self.pathE = pathE
#             self.pathN = pathN
#             self.pathChi = pathChi
#             self.pathLeftBoundaryE = pathLeftBoundaryE
#             self.pathLeftBoundaryN = pathLeftBoundaryN
#             self.pathRightBoundaryE = pathRightBoundaryE
#             self.pathRightBoundaryN = pathRightBoundaryN
#
#             pass
#
#     return path
#
#

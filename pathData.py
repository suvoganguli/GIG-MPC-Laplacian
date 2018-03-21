import numpy as np
from laplacianPlanner import *
from obstacleData import *
from utils import *

def pathInitData(case, startPoint, endPoint, pathWidth, obstacle = None, grid = None):

    # ---------------------------------------------------------
    # Default Path

    if case == 'default':
        pathDist = np.linalg.norm(endPoint - startPoint)
        pathSectionLength = 10.0
        nPathSections = int(pathDist/pathSectionLength)
        pathSectionLengths = pathSectionLength * np.ones(nPathSections)

        deltaPathLength = pathDist - pathSectionLength * nPathSections
        if deltaPathLength > 0:
            pathSectionLengths = np.append(pathSectionLengths, deltaPathLength)
            nPathSections = nPathSections + 1

        dE = endPoint[0] - startPoint[0]
        dN = endPoint[1] - startPoint[1]

        if dN != 0:
            Chi = np.arctan(dE/dN)  # (road chi is w.r.t +North axis)
        else:
            Chi = np.sign(dN)*np.pi/2

        npts = nPathSections + 1
        pathChi = Chi *  np.ones(npts)

        path = np.zeros([3,npts])

        path[0,0] = startPoint[0]
        path[1,0] = startPoint[1]

        for k in range(npts - 1):
            path[0, k + 1] = path[0, k] + pathSectionLengths[k] * np.sin(pathChi[k])
            path[1, k + 1] = path[1, k] + pathSectionLengths[k] * np.cos(pathChi[k])

        #pathWidth = 10.0

    # ---------------------------------------------------------
    # Calling Laplacian Planner

    if case == 'newpath':

        nE = grid.nE
        nN = grid.nN
        nU = grid.nU
        nU_low = grid.nU_low
        gridSize = grid.gridSize  # ft
        height = grid.height  # ft

        print(nE,nN,nU)

        if nE > 16:
            sf_E = float(nE)/16
            nE = int(nE/sf_E)
            obstacle.E = np.array([int(obstacle.E/sf_E)])
            obstacle.w = np.array([int(obstacle.w/sf_E)])
            startPoint[0] = startPoint[0]/sf_E
            endPoint[0] = endPoint[0] / sf_E
        else:
            sf_E = 1.0

        if nN > 128:
            sf_N = float(nN)/128
            nN = int(nN/sf_N)
            obstacle.N = np.array([int(obstacle.N/sf_N)])
            obstacle.l = np.array([int(obstacle.l/sf_N)])
            startPoint[1] = startPoint[1]/sf_N
            endPoint[1] = endPoint[1] / sf_N
        else:
            sf_N = 1.0

        if nU > 8:
            sf_U = float(nU)/8
            nU = int(nU/sf_U)
            nU_low = int(nU/sf_U)
            startPoint[2] = startPoint[2]/sf_U
            endPoint[2] = endPoint[2] / sf_U
        else:
            sf_U = 1.0

        print(nE, nN, nU)

        slow_convergence_test = 1

        startPoint_ = np.append(startPoint, height)
        endPoint_ = np.append(endPoint, height)

        obstacleData = createObstacleData(nE, nN, nU, gridSize, obstacle)

        path, not_converged = laplacian( startPoint_, endPoint_, nE, nN, nU, nU_low, obstacleData, slow_convergence_test )

        pathE = path[0, :] * gridSize  # ft
        pathN = path[1, :] * gridSize  # ft
        pathU = path[2, :] * gridSize  # ft

        pathE = path[0, :] * sf_E  # ft
        pathN = path[1, :] * sf_N  # ft
        pathU = path[2, :] * sf_U  # ft

        npts = len(pathE)
        nPathSections = npts -1

        pathChi = np.zeros(nPathSections)
        pathSectionLengths = np.zeros(nPathSections)

        for k in range(len(pathSectionLengths)):
            dE = pathE[k+1] - pathE[k]
            dN = pathN[k+1] - pathN[k]
            if dN != 0:
                pathChi[k] = np.arctan(dE / dN)  # (road chi is w.r.t +North axis)
            else:
                pathChi[k] = np.sign(dN) * np.pi / 2
            pathSectionLengths[k] = np.sqrt(dE**2 + dN**2)

        # Set the last point heading as that of the previous point
        pathChi = np.append(pathChi, pathChi[-1])

        #pathWidth = 10.0

    # ---------------------------------------------------------

    data = {'case':case,
            'pathStartPoint': startPoint,
            'pathEndPoint': endPoint,
            'pathLaplacian': path, # E, N, U - ft, ft, ft
            'pathChi': pathChi,
            'pathWidth': pathWidth,
            'pathSectionLengths': pathSectionLengths
            }

    return data


# def pathInputData(case):
#
#     nPathSections = 50
#     pathSectionLengths = 10*np.ones(nPathSections) # ft
#     pathSectionCurvatures = np.zeros(nPathSections) # ft
#     pathWidth = 10.0 # ft
#     data = {'case':case,
#             'pathStart': [0.0, 0.0, np.pi/2], # ft, ft, rad (road theta is w.r.t +East axis)
#             'pathWidth': pathWidth,
#             'nPathSections': nPathSections,
#             'pathSectionLengths': pathSectionLengths,
#             'pathSectionCurvatures': pathSectionCurvatures,
#             }
#
#     return data


def pathDetailedData(pathInputData):
    pathLaplacian = pathInputData['pathLaplacian']
    pathChi = pathInputData['pathChi']
    pathWidth = pathInputData['pathWidth']
    pathSectionLengths = pathInputData['pathSectionLengths']
    pathStartPoint = pathInputData['pathStartPoint']
    pathEndPoint = pathInputData['pathEndPoint']

    #  Set the initial location of the pathLP section
    E0 = pathLaplacian[0,0] # ft
    N0 = pathLaplacian[1,0] # ft
    theta0 = np.pi/2 - pathChi[0] # rad

    # Waypoint Data
    # deltaWP = 1 # ft, Waypoint separation distance

    # Number of intermediate waypoints
    n = 10

    # pathWidth/2
    d = pathWidth/2

    # Create zero vectors to hold full path definition
    E_full = np.empty(0)
    N_full = np.empty(0)
    Theta_full = np.empty(0)
    PathLeftE_full = np.empty(0)
    PathLeftN_full = np.empty(0)
    PathRightE_full = np.empty(0)
    PathRightN_full = np.empty(0)
    E_endpoints = np.empty(0)
    N_endpoints = np.empty(0)
    Theta_endpoints = np.empty(0)

    # Loop through each set of parameters (each is a path section)

    Theta = []

    nPathSections = len(pathSectionLengths)

    for i in range(nPathSections):

        secLength = pathSectionLengths[i]

        #n = np.int(secLength / deltaWP)
        #if n != int(n):
        #    print('secLenth must be an integer multiple of deltaWP')

        deltaWP = secLength / n

        path_dChi = pathChi[i+1] - pathChi[i]
        path_tht = - path_dChi

        if n > 0:
            path_dtht = path_tht / n
            Theta = theta0 + path_dtht * (np.arange(n+1))
        else:
            Theta = np.array([theta0])

        if (Theta[-1] * 180 / np.pi > 269) and (Theta[-1] * 180 / np.pi < 270):
            path_dtht2 = [270 * np.pi / 180 - Theta[-1]]
            Theta[-1] = Theta[-1] + path_dtht2

        nWP = len(Theta)

        E = np.zeros(nWP)
        N = np.zeros(nWP)
        PathLeftE = np.zeros(nWP)
        PathLeftN = np.zeros(nWP)
        PathRightE = np.zeros(nWP)
        PathRightN = np.zeros(nWP)

        for j in range(nWP):
            theta = Theta[j]
            E[j] = E0
            N[j] = N0
            PathLeftE[j] = E0 - d * np.sin(theta)
            PathLeftN[j] = N0 + d * np.cos(theta)
            PathRightE[j] = E0 + d * np.sin(theta)
            PathRightN[j] = N0 - d * np.cos(theta)

            E0 = E0 + deltaWP * np.cos(theta)
            N0 = N0 + deltaWP * np.sin(theta)

        # Debug
        #E_err = E[-1] - path.T[i+1, 0]
        #N_err = N[-1] - path.T[i+1, 1]
        #print(i, E_err, N_err)
        #if abs(N_err) > 5:
        #    None

        # Add the section to the complete path defintion
        E_full = np.concatenate((E_full, E[:-1]))
        N_full = np.concatenate((N_full, N[:-1]))
        Theta_full = np.concatenate((Theta_full, Theta[:-1]))
        PathLeftE_full = np.concatenate((PathLeftE_full, PathLeftE[:-1]))
        PathLeftN_full = np.concatenate((PathLeftN_full, PathLeftN[:-1]))
        PathRightE_full = np.concatenate((PathRightE_full, PathRightE[:-1]))
        PathRightN_full = np.concatenate((PathRightN_full, PathRightN[:-1]))

        # Reset the starting points for the next section
        E0 = E[nWP-1]
        N0 = N[nWP-1]
        theta0 = Theta[nWP-1]

        # Store the start points of the section
        E_endpoints = np.concatenate((E_endpoints, [E[0]]))
        N_endpoints = np.concatenate((N_endpoints, [N[0]]))
        Theta_endpoints = np.concatenate((Theta_endpoints, [Theta[0]]))

    E_full = np.concatenate((E_full, E[-1:]))
    N_full = np.concatenate((N_full, N[-1:]))
    Theta_full = np.concatenate((Theta_full, Theta[-1:]))
    PathLeftE_full = np.concatenate((PathLeftE_full, PathLeftE[-1:]))
    PathLeftN_full = np.concatenate((PathLeftN_full, PathLeftN[-1:]))
    PathRightE_full = np.concatenate((PathRightE_full, PathRightE[-1:]))
    PathRightN_full = np.concatenate((PathRightN_full, PathRightN[-1:]))

    # Store the end point of the last section
    E_endpoints = np.concatenate((E_endpoints, [E[-1]]))
    N_endpoints = np.concatenate((N_endpoints, [N[-1]]))
    Theta_endpoints = np.concatenate((Theta_endpoints, [Theta[-1]]))

    # Define final variables for storage in class

    CenterEndPointsE = E_endpoints
    CenterEndPointsN = N_endpoints
    RightEndPointsE = E_endpoints + pathWidth / 2 * np.sin(Theta_endpoints)
    RightEndPointsN = N_endpoints - pathWidth / 2 * np.cos(Theta_endpoints)
    LeftEndPointsE = E_endpoints - pathWidth / 2 * np.sin(Theta_endpoints)
    LeftEndPointsN = N_endpoints + pathWidth/2 * np.cos(Theta_endpoints)


    # Save lane data in classes
    class path(object):
        def __init__(self):
            self.nPathSections = nPathSections
            self.pathSectionLengths = pathSectionLengths
            self.pathLaplacian = pathLaplacian
            self.E = E_full
            self.N = N_full
            self.Theta = Theta_full
            self.PathLeftBoundaryE = PathLeftE_full
            self.PathLeftBoundaryN = PathLeftN_full
            self.PathRightBoundaryE = PathRightE_full
            self.PathRightBoundaryN = PathRightN_full
            self.PathCenterEndPointsE = CenterEndPointsE
            self.PathCenterEndPointsN = CenterEndPointsN
            self.PathRightEndPointsE = RightEndPointsE
            self.PathRightEndPointsN = RightEndPointsN
            self.PathLeftEndPointsE = LeftEndPointsE
            self.PathLeftEndPointsN = LeftEndPointsN
            self.Theta_endpoints = Theta_endpoints
            self.PathStartPoint = pathStartPoint
            self.PathEndPoint = pathEndPoint
            pass

    return path

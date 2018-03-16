from utils import *
import matplotlib.path as mplPath

def obstacleInfo(obstaclePresent, obstacleE, obstacleN, obstacleChi, obstacleWidth, obstacleLength):

    class obstacle():
        def __init__(self):
            self.Present = obstaclePresent
            self.E = obstacleE
            self.N = obstacleN
            self.Chi = obstacleChi
            self.w = obstacleWidth
            self.l = obstacleLength
            pass

    return obstacle

def createObstacleData(nE, nN, nU, gridsize, obstacle):

    obstacleOnGrid = np.zeros([nE, nN, nU])
    n = len(obstacle.E)

    for i in range(n):
        EGrid = np.floor( obstacle.E[i] / gridsize )
        NGrid = np.floor( obstacle.N[i] / gridsize )
        wGrid = np.ceil( obstacle.w[i] / gridsize )
        lGrid = np.ceil( obstacle.l[i] / gridsize )

        EGrid = np.int(EGrid)
        NGrid = np.int(NGrid)
        wGrid = np.int(wGrid)
        lGrid = np.int(lGrid)

        for j in range(wGrid):
            for k in range(lGrid):
                obstacleOnGrid[EGrid + j, NGrid + k,:] = 1

    # floor is an obstaclce
    obstacleOnGrid[:,:,0] = 1

    return obstacleOnGrid


def detectObstacle(x0, detectionWindow, obstacle):

    nObs = obstacle.E.size

    # corners of detection window
    E = x0[0]
    N = x0[1]
    Chi = x0[3]

    l = detectionWindow['L']
    w = detectionWindow['W']

    p1Win = np.array([E - w/2, N])
    p2Win = np.array([E + w/2, N])
    p3Win = np.array([E + w/2, N + l])
    p4Win = np.array([E - w/2, N + l])

    p1Win = rotate(p1Win, Chi)
    p2Win = rotate(p2Win, Chi)
    p3Win = rotate(p3Win, Chi)
    p4Win = rotate(p4Win, Chi)

    bbPath = mplPath.Path(np.array([p1Win, p2Win, p3Win, p4Win]))

    # corners of obstacle

    detected = False
    for k in range(nObs):
        p1Obs = np.array([obstacle.E[k] - obstacle.w[k]/2, obstacle.N[k]])
        p2Obs = np.array([obstacle.E[k] + obstacle.w[k]/2, obstacle.N[k]])
        p3Obs = np.array([obstacle.E[k] + obstacle.w[k]/2, obstacle.N[k] + obstacle.l[k]])
        p4Obs = np.array([obstacle.E[k] - obstacle.w[k]/2, obstacle.N[k] + obstacle.l[k]])

        p1Obs = rotate(p1Obs, obstacle.Chi[k])
        p2Obs = rotate(p2Obs, obstacle.Chi[k])
        p3Obs = rotate(p3Obs, obstacle.Chi[k])
        p4Obs = rotate(p4Obs, obstacle.Chi[k])

        # Current algorithm searches for detection of obstacle corners only in
        # detection window. This will be improved later on
        det1 = bbPath.contains_point(p1Obs)
        det2 = bbPath.contains_point(p2Obs)
        det3 = bbPath.contains_point(p3Obs)
        det4 = bbPath.contains_point(p4Obs)
        detected = detected or det1 or det2 or det3 or det4

    return detected


def getObstacleData(obstacle, obstacleIdx):

    class obstacleIdxData(object):

        def __init__(self):
            self.Present = obstacle.Present
            self.E = np.array([obstacle.E[obstacleIdx]])
            self.N = np.array([obstacle.N[obstacleIdx]])
            self.w = np.array([obstacle.w[obstacleIdx]])
            self.l = np.array([obstacle.l[obstacleIdx]])
            self.Chi = np.array([obstacle.Chi[obstacleIdx]])
            pass

    return obstacleIdxData


def remainingObstacle(obstacle):

    class obstacleRemainingData(object):
        def __init__(self):

            self.Present = obstacle.Present

            n = obstacle.E.size
            if n > 1:
                self.E = obstacle.E[1:]
                self.N = obstacle.N[1:]
                self.w = obstacle.w[1:]
                self.l = obstacle.l[1:]
                self.Chi = obstacle.Chi[1:]
            else:
                self.E = np.array([obstacle.E[1:]])
                self.N = np.array([obstacle.N[1:]])
                self.w = np.array([obstacle.w[1:]])
                self.l = np.array([obstacle.l[1:]])
                self.Chi = np.array([obstacle.Chi[1:]])
            pass

    return obstacleRemainingData

def getCurrentObstacle(obstacle):

    class obstacleCurrentData(object):
        def __init__(self):

            self.Present = obstacle.Present
            self.E = np.array([obstacle.E[0]])
            self.N = np.array([obstacle.N[0]])
            self.w = np.array([obstacle.w[0]])
            self.l = np.array([obstacle.l[0]])
            self.Chi = np.array([obstacle.Chi[0]])

            pass

    return obstacleCurrentData

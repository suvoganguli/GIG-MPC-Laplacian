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

def detectObstacle(x0, detectionWindow, obstacle):

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
    p1Obs = np.array([obstacle.E - obstacle.w/2, obstacle.N])
    p2Obs = np.array([obstacle.E + obstacle.w/2, obstacle.N])
    p3Obs = np.array([obstacle.E + obstacle.w/2, obstacle.N + obstacle.l])
    p4Obs = np.array([obstacle.E - obstacle.w/2, obstacle.N + obstacle.l])

    p1Obs = rotate(p1Obs, obstacle.Chi)
    p2Obs = rotate(p2Obs, obstacle.Chi)
    p3Obs = rotate(p3Obs, obstacle.Chi)
    p4Obs = rotate(p4Obs, obstacle.Chi)

    # Current algorithm searches for detection of obstacle corners only in
    # detection window. This will be improved later on
    det1 = bbPath.contains_point(p1Obs)
    det2 = bbPath.contains_point(p2Obs)
    det3 = bbPath.contains_point(p3Obs)
    det4 = bbPath.contains_point(p4Obs)
    detected = det1 or det2 or det3 or det4

    return detected
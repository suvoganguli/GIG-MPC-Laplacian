def obstacleInfo(obstaclePresent,obstacleE,obstacleN,obstacleWidth,obstacleLength):

    class obstacle():
        def __init__(self):
            self.obstaclePresent = obstaclePresent
            self.obstacleE = obstacleE
            self.obstacleN = obstacleN
            self.obstacleWidth = obstacleWidth
            self.obstacleLength = obstacleLength
            pass

    return obstacle
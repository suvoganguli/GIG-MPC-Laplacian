from problemData import *
from pathData import *
from pathCosts import *
from pathCons import *
from pathLines import *
from utils import *

def pathInfo(case, obstacle=None):

    # Path initial data
    if case == 'newpath':
        pathData = pathInitData(case, startPoint, endPoint, obstacle)

    elif case == 'default':
        pathData = pathInitData(case, startPoint, endPoint)

    else:
        pathData = None

    # Path detailed data
    pathClass = pathDetailedData(pathData)
    path = pathClass()

    # Cost data
    costAlongPathClass, costAcrossPathClass = pathCostData(path)
    costAlongPath = costAlongPathClass()
    costAcrossPath = costAcrossPathClass()

    # Constraint data
    consPathClass = pathConsData(path)
    consPath = consPathClass()

    # Road line data
    alongPathLinesClass, acrossPathLinesClass = pathLines(costAlongPath, costAcrossPath, consPath)
    alongPathLines = alongPathLinesClass()
    acrossPathLines = acrossPathLinesClass()

    class lanes():
        def __init__(self,x=None,y=None):
            self.pathSectionLengths = pathData['pathSectionLengths']
            self.pathWidth = pathData['pathWidth']
            self.path = path
            self.costAlongPath = costAlongPath
            self.costAcrossPath = costAcrossPath
            self.consPath = consPath
            self.alongPathLines = alongPathLines
            self.acrossPathLines = acrossPathLines
            pass

        def insideRoadSegment(self, x, y, lane1Lines, lane2Lines, acrossLines):

            AR_Lane1 = lane1Lines.AR_Lane
            BR_Lane1 = lane1Lines.BR_Lane
            CR_Lane1 = lane1Lines.CR_Lane
            AL_Lane1 = lane1Lines.AL_Lane
            BL_Lane1 = lane1Lines.BL_Lane
            CL_Lane1 = lane1Lines.CL_Lane

            AR_Lane2 = lane2Lines.AR_Lane
            BR_Lane2 = lane2Lines.BR_Lane
            CR_Lane2 = lane2Lines.CR_Lane
            AL_Lane2 = lane2Lines.AL_Lane
            BL_Lane2 = lane2Lines.BL_Lane
            CL_Lane2 = lane2Lines.CL_Lane

            D1 = acrossLines.D1
            E1 = acrossLines.E1
            F1 = acrossLines.F1
            D2 = acrossLines.D2
            E2 = acrossLines.E2
            F2 = acrossLines.F2

            n = len(lane1Lines.A_Lane)

            idx_Obstacle = -1
            laneNo = -1
            for k in range(n):

                insideLane1 = insideBox(x, y,
                                   AR_Lane1[k], BR_Lane1[k], CR_Lane1[k], AL_Lane1[k], BL_Lane1[k], CL_Lane1[k],
                                   D1[k], E1[k], F1[k], D2[k], E2[k], F2[k])
                insideLane2 = insideBox(x, y,
                                   AR_Lane2[k], BR_Lane2[k], CR_Lane2[k], AL_Lane2[k], BL_Lane2[k], CL_Lane2[k],
                                   D1[k], E1[k], F1[k], D2[k], E2[k], F2[k])

                if insideLane1 == True:
                    idx_Obstacle = k
                    laneNo = 1
                    break
                elif insideLane2 == True:
                    idx_Obstacle = k
                    laneNo = 2
                    break

            return idx_Obstacle, laneNo

    return lanes
from roadData import *
from roadCosts import *
from roadCons import *
from roadLines import *
from utils import *

def roadLanes(case):
    roadData = roadInputData(case)
    lane1Class, lane2Class = roadDetailedData(roadData)
    lane1 = lane1Class()
    lane2 = lane2Class()

    # Cost data
    costLane1Class, costLane2Class, costAcrossClass = roadCostData(lane1, lane2)
    costLane1 = costLane1Class()
    costLane2 = costLane2Class()
    costAcross = costAcrossClass()

    # Constraint data
    consLane1Class, consLane2Class = roadConsData(lane1, lane2)
    consLane1 = consLane1Class()
    consLane2 = consLane2Class()

    # Road line data
    lane1LinesClass, lane2LinesClass, acrossLinesClass = roadLineData(costLane1, costLane2, costAcross, consLane1, consLane2)
    lane1Lines = lane1LinesClass()
    lane2Lines = lane2LinesClass()
    acrossLines = acrossLinesClass()


    class lanes():
        def __init__(self,x=None,y=None):
            self.numberOfLanes = roadData['numberOfLanes']
            self.roadSegmentLengths = roadData['roadSegmentLengths']
            self.laneWidth = roadData['laneWidth']
            self.lane1 = lane1
            self.lane2 = lane2
            self.costLane1 = costLane1
            self.costLane2 = costLane2
            self.costAcross = costAcross
            self.consLane1 = consLane1
            self.consLane2 = consLane2
            self.lane1Lines = lane1Lines
            self.lane2Lines = lane2Lines
            self.acrossLines = acrossLines
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
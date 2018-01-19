import numpy as np

def roadLineData(costLane1, costLane2, costAcross, consLane1, consLane2):
    npts = len(costLane1.costLane_b)

    A_Lane1 = np.zeros(npts)
    B_Lane1 = np.zeros(npts)
    C_Lane1 = np.zeros(npts)
    AR_Lane1 = np.zeros(npts)
    BR_Lane1 = np.zeros(npts)
    CR_Lane1 = np.zeros(npts)
    AL_Lane1 = np.zeros(npts)
    BL_Lane1 = np.zeros(npts)
    CL_Lane1 = np.zeros(npts)

    for k in range(npts):

        A_Lane1[k] = costLane1.costLane_A[k,0]
        B_Lane1[k] = costLane1.costLane_A[k,1]
        C_Lane1[k] = costLane1.costLane_b[k]

        AR_Lane1[k] = consLane1.consLane_AR[k,0]
        BR_Lane1[k] = consLane1.consLane_AR[k,1]
        CR_Lane1[k] = consLane1.consLane_bR[k]

        AL_Lane1[k] = consLane1.consLane_AL[k,0]
        BL_Lane1[k] = consLane1.consLane_AL[k,1]
        CL_Lane1[k] = consLane1.consLane_bL[k]


    class lane1Lines:
        def __init__(self):
            self.A_Lane = A_Lane1
            self.B_Lane = B_Lane1
            self.C_Lane = C_Lane1
            self.AR_Lane = AR_Lane1
            self.BR_Lane = BR_Lane1
            self.CR_Lane = CR_Lane1
            self.AL_Lane = AL_Lane1
            self.BL_Lane = BL_Lane1
            self.CL_Lane = CL_Lane1

    A_Lane2 = np.zeros(npts)
    B_Lane2 = np.zeros(npts)
    C_Lane2 = np.zeros(npts)
    AR_Lane2 = np.zeros(npts)
    BR_Lane2 = np.zeros(npts)
    CR_Lane2 = np.zeros(npts)
    AL_Lane2 = np.zeros(npts)
    BL_Lane2 = np.zeros(npts)
    CL_Lane2 = np.zeros(npts)


    for k in range(npts):

        A_Lane2[k] = costLane2.costLane_A[k,0]
        B_Lane2[k] = costLane2.costLane_A[k,1]
        C_Lane2[k] = costLane2.costLane_b[k]

        AR_Lane2[k] = consLane2.consLane_AR[k,0]
        BR_Lane2[k] = consLane2.consLane_AR[k,1]
        CR_Lane2[k] = consLane2.consLane_bR[k]

        AL_Lane2[k] = consLane2.consLane_AL[k,0]
        BL_Lane2[k] = consLane2.consLane_AL[k,1]
        CL_Lane2[k] = consLane2.consLane_bL[k]

    class lane2Lines:
        def __init__(self):
            self.A_Lane = A_Lane2
            self.B_Lane = B_Lane2
            self.C_Lane = C_Lane2
            self.AR_Lane = AR_Lane2
            self.BR_Lane = BR_Lane2
            self.CR_Lane = CR_Lane2
            self.AL_Lane = AL_Lane2
            self.BL_Lane = BL_Lane2
            self.CL_Lane = CL_Lane2

    D1 = np.zeros(npts)
    E1 = np.zeros(npts)
    F1 = np.zeros(npts)
    D2 = np.zeros(npts)
    E2 = np.zeros(npts)
    F2 = np.zeros(npts)

    for k in range(npts):
        D1[k] = costAcross.costAcross_A[k,0]
        E1[k] = costAcross.costAcross_A[k,1]
        F1[k] = costAcross.costAcross_b[k]
        D2[k] = costAcross.costAcross_A[k+1,0]
        E2[k] = costAcross.costAcross_A[k+1,1]
        F2[k] = costAcross.costAcross_b[k+1]

    class acrossLines:
        def __init__(self):
            self.D1 = D1
            self.E1 = E1
            self.F1 = F1
            self.D2 = D2
            self.E2 = E2
            self.F2 = F2

    return lane1Lines, lane2Lines, acrossLines

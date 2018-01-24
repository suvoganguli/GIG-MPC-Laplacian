import numpy as np

def pathLines(costAlongPath, costAcrossPath, consPath):
    npts = len(costAlongPath.b)

    A = np.zeros(npts)
    B = np.zeros(npts)
    C = np.zeros(npts)
    AR = np.zeros(npts)
    BR = np.zeros(npts)
    CR = np.zeros(npts)
    AL = np.zeros(npts)
    BL = np.zeros(npts)
    CL = np.zeros(npts)

    for k in range(npts):

        A[k] = costAlongPath.A[k,0]
        B[k] = costAlongPath.A[k,1]
        C[k] = costAlongPath.b[k]

        AR[k] = consPath.AR[k,0]
        BR[k] = consPath.AR[k,1]
        CR[k] = consPath.bR[k]

        AL[k] = consPath.AL[k,0]
        BL[k] = consPath.AL[k,1]
        CL[k] = consPath.bL[k]


    class alongPathLines:
        def __init__(self):
            self.A = A
            self.B = B
            self.C = C
            self.AR = AR
            self.BR = BR
            self.CR = CR
            self.AL = AL
            self.BL = BL
            self.CL = CL

    D1 = np.zeros(npts)
    E1 = np.zeros(npts)
    F1 = np.zeros(npts)
    D2 = np.zeros(npts)
    E2 = np.zeros(npts)
    F2 = np.zeros(npts)

    for k in range(npts):
        D1[k] = costAcrossPath.A[k,0]
        E1[k] = costAcrossPath.A[k,1]
        F1[k] = costAcrossPath.b[k]
        D2[k] = costAcrossPath.A[k+1,0]
        E2[k] = costAcrossPath.A[k+1,1]
        F2[k] = costAcrossPath.b[k+1]


    class acrossPathLines:
        def __init__(self):
            self.D1 = D1
            self.E1 = E1
            self.F1 = F1
            self.D2 = D2
            self.E2 = E2
            self.F2 = F2

    return alongPathLines, acrossPathLines

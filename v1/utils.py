import numpy as np
from matplotlib.patches import Polygon

def insideBox(x,y,AR,BR,CR,AL,BL,CL,D1,E1,F1,D2,E2,F2):

    chk1bool = False
    chk2bool = False
    chk3bool = False
    chk4bool = False

    chk1 = D1 * x + E1 * y - F1
    chk2 = D2 * x + E2 * y - F2
    chk3 = AR * x + BR * y - CR
    chk4 = AL * x + BL * y - CL

    myeps = 1e-3

    if chk1 >= -myeps:
        chk1bool = True
    if chk2 <= myeps:
        chk2bool = True
    if chk3 <= 0:
        chk3bool = True
    if chk4 >= 0:
        chk4bool = True

    if (chk1bool) and \
            (chk2bool) and \
            (chk3bool) and \
            (chk4bool):
        return True
    else:
        return False

def insideGap(x,y,AR,BR,CR,AL,BL,CL):

    chk1bool = False
    chk2bool = False

    chk1 = AR * x + BR * y - CR
    chk2 = AL * x + BL * y - CL

    myeps = 1e-3

    if chk1 >= -myeps:
        chk1bool = True
    if chk2 <= myeps:
        chk2bool = True

    if (chk1bool) and (chk2bool):
        return True
    else:
        return False

def changeAxis(x,y,theta):
    C = [[np.cos(theta), np.sin(theta)],[-np.sin(theta),np.cos(theta)]]
    X = np.array([x,y]).T
    Y = np.matmul(C,X)
    print(X)
    return Y[0], Y[1]


def intersect(laneLines, acrossLines, idx):
    a1 = laneLines.A_Lane[idx]
    b1 = laneLines.B_Lane[idx]
    c1 = laneLines.C_Lane[idx]

    a2 = acrossLines.D1[idx]
    b2 = acrossLines.E1[idx]
    c2 = acrossLines.F1[idx]

    determinant = a1*b2 - a2*b1

    if (determinant == 0):
        return []
    else:
        x = (c1*b2 - c2*b1) / determinant
        y = (a1*c2 - a2*c1) / determinant

    return x, y

def getPatch(Efc,Nfc,W,L,theta,fc):

    # create object with heading = 0 deg

    E1 = -W/2
    E2 = +W/2
    E3 = E2
    E4 = E1
    N1 = 0
    N2 = N1
    N3 = L
    N4 = N3
    E = np.array([E1,E2,E3,E4])
    N = np.array([N1,N2,N3,N4])

    # Rotate object

    ERot = np.zeros(4)
    NRot = np.zeros(4)
    C = np.array([[np.cos(theta), +np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
    for k in range(len(E)):
        p = np.array([E[k], N[k]])
        p = p[:,None]
        pRot = np.dot(C,p)
        #pRot = p
        ERot[k] = pRot[0] + Efc
        NRot[k] = pRot[1] + Nfc

    vertices = np.array([ERot, NRot])
    polygon = Polygon(vertices.T, facecolor=fc, alpha=0.75)

    return polygon

def distance(p1,p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]

    d = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return d

def shiftRotate(vec1,dvec,Chi):

    dcm = np.array([
        [np.cos(Chi), np.sin(Chi)],
        [-np.sin(Chi), np.cos(Chi)]
    ])
    vec1 = vec1[:, None]
    dvec = dvec[:, None]
    vec2 = vec1 + np.matmul(dcm, dvec)

    return vec2

def rotate(vec1,Chi):

    dcm = np.array([
        [np.cos(Chi), np.sin(Chi)],
        [-np.sin(Chi), np.cos(Chi)]
    ])
    vec1 = vec1[:, None]
    vec2 = np.matmul(dcm, vec1)

    return vec2
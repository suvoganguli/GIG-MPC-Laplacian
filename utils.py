import numpy as np
from matplotlib.patches import Polygon
import pickle

def getPosIdx(E, N, path, posIdx0 = None):

    posIdx = {'number': -1}

    AR_Path = path.alongPathLines.AR
    BR_Path = path.alongPathLines.BR
    CR_Path = path.alongPathLines.CR
    AL_Path = path.alongPathLines.AL
    BL_Path = path.alongPathLines.BL
    CL_Path = path.alongPathLines.CL

    D1 = path.acrossPathLines.D1
    E1 = path.acrossPathLines.E1
    F1 = path.acrossPathLines.F1
    D2 = path.acrossPathLines.D2
    E2 = path.acrossPathLines.E2
    F2 = path.acrossPathLines.F2

    nSections = len(D1)

    if posIdx0 == None:
        kvec = range(nSections)
    else:
        kvec = np.arange(posIdx0['number'], nSections, 1)


    for k in kvec:
        inbox = insideBox(E, N, AR_Path[k], BR_Path[k], CR_Path[k], AL_Path[k], BL_Path[k], CL_Path[k],
                          D1[k], E1[k], F1[k], D2[k], E2[k], F2[k])

        if inbox == True:
            posIdx['number'] = k
            return posIdx

    return posIdx


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

def intersect2(a1,b1,c1,a2,b2,c2):

    determinant = a1*b2 - a2*b1

    if (determinant == 0):
        return []
    else:
        x = (c1*b2 - c2*b1) / determinant
        y = (a1*c2 - a2*c1) / determinant

    return x, y


def getPatch(Elb,Nlb,W,L,theta,fc):

    # create object with heading = 0 deg

    E1 = 0
    E2 = W
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
        ERot[k] = pRot[0] + Elb
        NRot[k] = pRot[1] + Nlb

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
    vec1 = np.squeeze(vec1)
    vec1 = vec1[:, None]

    vec2 = np.matmul(dcm, vec1)

    return np.squeeze(vec2)

def createGrid(gridSize, lengthSpace, widthSpace, heightSpace):

    # always set height at the middle of the space to run Laplacian Planner without error
    height = heightSpace / 2  # ft

    nE = widthSpace / gridSize
    nN = lengthSpace / gridSize
    nU = heightSpace / gridSize
    nU_low = nU

    class grid():
        def __init__(self):
            self.nE = nE
            self.nN = nN
            self.nU = nU
            self.nU_low = nU_low
            self.height = height
            self.gridSize = gridSize # ft
            self.lengthSpace = lengthSpace # ft
            self.widthSpace = widthSpace  # ft
            self.heightSpace = heightSpace  # ft
            pass

    return grid


def getColumns(inFile, delim=" ", header=True):     # delim="\t"
    """
    Get columns of data from inFile. The order of the rows is respected

    :param inFile: column file separated by delim
    :param header: if True the first line will be considered a header line
    :returns: a tuple of 2 dicts (cols, indexToName). cols dict has keys that
    are headings in the inFile, and values are a list of all the entries in that
    column. indexToName dict maps column index to names that are used as keys in
    the cols dict. The names are the same as the headings used in inFile. If
    header is False, then column indices (starting from 0) are used for the
    heading names (i.e. the keys in the cols dict)
    """
    cols = {}
    indexToName = {}
    for lineNum, line in enumerate(inFile):
        if lineNum == 0:
            headings = line.split(delim)
            i = 0
            for heading in headings:
                heading = heading.strip()
                if header:
                    cols[heading] = []
                    indexToName[i] = heading
                else:
                    # in this case the heading is actually just a cell
                    cols[i] = [heading]
                    indexToName[i] = i
                i += 1
        else:
            cells = line.split(delim)
            i = 0
            for cell in cells:
                cell = cell.strip()
                cols[indexToName[i]] += [cell]
                i += 1

    return cols, indexToName

def savepkl(obj, file_pkl):
    with open(file_pkl, 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)
        #pickle.dump(obj, f, pickle.0)

def loadpkl(file_pkl):
    with open(file_pkl, 'rb') as f:
        return pickle.load(f)

def makePathObj(pdata, path, obstacle):
    pathObj = { 'PathE': path.pathData.E,
                'PathN': path.pathData.N,
                'PathStartPoint': path.pathData.PathStartPoint,
                'PathEndPoint': path.pathData.PathEndPoint,
                'PathRightEndPointsE': path.pathData.PathRightEndPointsE,
                'PathRightEndPointsN': path.pathData.PathRightEndPointsN,
                'PathLeftEndPointsE': path.pathData.PathLeftEndPointsE,
                'PathLeftEndPointsN': path.pathData.PathLeftEndPointsN,
                'PathCenterEndPointsE': path.pathData.PathCenterEndPointsE,
                'PathCenterEndPointsN': path.pathData.PathCenterEndPointsN,
                'PathThetaEndpoints': path.pathData.Theta_endpoints,
                'PathDeltaYRoad':  pdata.delta_yRoad,
                'PathWidth': pdata.pathWidth,
                'ObstacleE': obstacle.E,
                'ObstacleN': obstacle.N,
                'ObstacleW': obstacle.w,
                'ObstacleL': obstacle.l,
                'ObstacleChi': obstacle.Chi,
            }

    return pathObj

def addCurrentPointToPath(path, startPoint, Chi):

    Theta = np.pi/2 - Chi

    path.pathData.E = np.append(startPoint[0], path.pathData.E)
    path.pathData.N = np.append(startPoint[1], path.pathData.N)
    path.pathData.PathStartPoint = startPoint

    RightEndPointE = startPoint[0] + path.pathWidth / 2 * np.sin(Theta)
    RightEndPointN = startPoint[1] - path.pathWidth / 2 * np.cos(Theta)
    LeftEndPointE = startPoint[0] - path.pathWidth / 2 * np.sin(Theta)
    LeftEndPointN = startPoint[1] + path.pathWidth / 2 * np.cos(Theta)

    path.pathData.PathRightEndPointsE = np.append(RightEndPointE, path.pathData.PathRightEndPointsE)
    path.pathData.PathRightEndPointsN = np.append(RightEndPointN, path.pathData.PathRightEndPointsN)
    path.pathData.PathLeftEndPointsE = np.append(LeftEndPointE, path.pathData.PathLeftEndPointsE)
    path.pathData.PathLeftEndPointsN = np.append(LeftEndPointN, path.pathData.PathLeftEndPointsN)

    path.pathData.PathCenterEndPointsE = np.append(startPoint[0], path.pathData.PathCenterEndPointsE)
    path.pathData.PathCenterEndPointsN = np.append(startPoint[1], path.pathData.PathCenterEndPointsN)

    path.pathData.Theta_endpoints = np.append(Theta, path.pathData.Theta_endpoints)

    return path


# def scalePath(path, sf_E, sf_N):
#     path.pathData.E = path.pathData.E * sf_E
#     path.pathData.N = path.pathData.N * sf_N
#     path.pathData.PathCenterEndPointsE = path.pathData.PathCenterEndPointsE * sf_E
#     path.pathData.PathCenterEndPointsN = path.pathData.PathCenterEndPointsN * sf_N
#     path.pathData.PathLeftEndPointsE = path.pathData.PathLeftEndPointsE * sf_E
#     path.pathData.PathLeftEndPointsN = path.pathData.PathLeftEndPointsN * sf_N
#     path.pathData.PathRightEndPointsE = path.pathData.PathRightEndPointsE * sf_E
#     path.pathData.PathRightEndPointsN = path.pathData.PathRightEndPointsN * sf_N
#     path.pathData.PathStartPoint

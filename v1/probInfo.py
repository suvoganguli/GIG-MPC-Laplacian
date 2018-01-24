from problemData import *
from utils import *

def system(uk, xk, T):

    if nstates == 2:
        # x0 = E, x1 = N
        # u0 = V, u1 = Chi

        xkp1 = [0, 0]
        xkp1[0] = xk[0] + T * uk[0] * np.sin(uk[1]) # Edot
        xkp1[1] = xk[1] + T * uk[0] * np.cos(uk[1]) # Ndot

        # xkp1 = [0, 0]
        # xkp1[0] = xk[0] + T * V_cmd * np.sin(uk[1]) # Edot
        # xkp1[1] = xk[1] + T * V_cmd * np.cos(uk[1]) # Ndot


    elif nstates == 4:
        # x0 = E, x1 = N, x2 = V, x3 = Chi
        # u0 = Vdot, u1 = Chidot

        xkp1 = [0, 0, 0, 0]
        xkp1[0] = xk[0] + T * xk[2] * np.sin(xk[3]) # Edot
        xkp1[1] = xk[1] + T * xk[2] * np.cos(xk[3]) # Ndot
        xkp1[2] = xk[2] + T * uk[0]                 # Vdot
        xkp1[3] = xk[3] + T * uk[1]                 # Chidot

        # xkp1 = [0, 0, 0, 0]
        # xkp1[0] = xk[0] + T * V_cmd * np.sin(xk[3]) # Edot
        # xkp1[1] = xk[1] + T * V_cmd * np.cos(xk[3]) # Ndot
        # xkp1[2] = xk[2] + T * uk[0]                 # Vdot
        # xkp1[3] = xk[3] + T * uk[1]                 # Chidot

    elif nstates == 6:
        # x0 = E, x1 = N, x2 = V, x3 = Chi, x5 = Vdot, x6 = Chidot
        # u0 = Vddot, u1 = Chiddot

        xkp1 = [0 ,0, 0, 0, 0, 0]
        xkp1[0] = xk[0] + T * xk[2] * np.sin(xk[3])     # Edot
        xkp1[1] = xk[1] + T * xk[2] * np.cos(xk[3])     # Ndot
        xkp1[2] = xk[2] + T * xk[4]                     # Vdot
        xkp1[3] = xk[3] + T * xk[5]                     # Chidot
        xkp1[4] = xk[4] + T * uk[0]                     # Vddot
        xkp1[5] = xk[5] + T * uk[1]                     # Chiddot

        # xkp1 = [0 ,0, 0, 0, 0, 0]
        # xkp1[0] = xk[0] + T * V_cmd * np.sin(xk[3])     # Edot
        # xkp1[1] = xk[1] + T * V_cmd * np.cos(xk[3])     # Ndot
        # xkp1[2] = xk[2] + T * xk[4]                     # Vdot
        # xkp1[3] = xk[3] + T * xk[5]                     # Chidot
        # xkp1[4] = xk[4] + T * uk[0]                     # Vddot
        # xkp1[5] = xk[5] + T * uk[1]                     # Chiddot


    return xkp1


def runningCosts(u, x, t0, path, obstacle, posIdx=None):

    A = path.alongPathLines.A
    B = path.alongPathLines.B
    C = path.alongPathLines.C
    AR = path.alongPathLines.AR
    BR = path.alongPathLines.BR
    CR = path.alongPathLines.CR
    AL = path.alongPathLines.AL
    BL = path.alongPathLines.BL
    CL = path.alongPathLines.CL

    D1 = path.acrossPathLines.D1
    E1 = path.acrossPathLines.E1
    F1 = path.acrossPathLines.F1
    D2 = path.acrossPathLines.D2
    E2 = path.acrossPathLines.E2
    F2 = path.acrossPathLines.F2

    nSections = len(D1)

    if posIdx == None:
        kvec = range(nSections)
    else:
        kvec = np.arange(posIdx['number'], nSections, 1)

    for k in kvec:

        cost = 0.0

        inbox = insideBox(x[0], x[1], AR[k], BR[k], CR[k], AL[k], BL[k], CL[k],
                     D1[k], E1[k], F1[k], D2[k], E2[k], F2[k])

        # inside road segment
        if inbox == True:

            if obstacle.obstaclePresent == False:
                a = A[k]
                b = B[k]
                c = C[k]
            else:
                print("TBD")

            if nstates == 6:
                cost_p = W_P * (a * x[0] + b * x[1] - c) ** 2 / (a ** 2 + b ** 2)
                cost_v = W_V * (V_cmd - x[2])**2
                cost_vddot = W_Vddot * u[0]**2
                cost_Chiddot = W_Chiddot * (u[1]*180/np.pi)**2
                cost_x = cost_p + cost_v
                cost_u = cost_vddot + cost_Chiddot
                cost = cost_x + cost_u

            elif nstates == 4:
                cost_p = W_P * (a * x[0] + b * x[1] - c) ** 2 / (a ** 2 + b ** 2)
                cost_v = W_V * (V_cmd - x[2])**2
                cost_vdot = W_Vdot * u[0]**2
                cost_Chidot = W_Chidot * (u[1]*180/np.pi)**2
                cost_x = cost_p + cost_v
                cost_u = cost_vdot + cost_Chidot
                cost = cost_x + cost_u

            return cost

        else:
            cost = 1e6

    return cost


def runningCons(u, x, t0, path, obstacle, posIdx=None):

    cons = [0,0] #[1,-1]  will lead to both constraints false in nlp.py
    yDist = np.zeros(2) # with respect to road axis (x - along, y - across)
    found_sol = False

    AR = path.alongPathLines.AR
    BR = path.alongPathLines.BR
    CR = path.alongPathLines.CR
    AL = path.alongPathLines.AL
    BL = path.alongPathLines.BL
    CL = path.alongPathLines.CL

    D1 = path.acrossPathLines.D1
    E1 = path.acrossPathLines.E1
    F1 = path.acrossPathLines.F1
    D2 = path.acrossPathLines.D2
    E2 = path.acrossPathLines.E2
    F2 = path.acrossPathLines.F2

    nSections = len(D1)
    if posIdx == None:
        kvec = range(nSections)
    else:
        kvec = np.arange(posIdx['number'], nSections, 1)

    for k in kvec:

        inbox = insideBox(x[0], x[1], AR[k], BR[k], CR[k], AL[k], BL[k], CL[k],
                     D1[k], E1[k], F1[k], D2[k], E2[k], F2[k])

        # inside road segment
        if inbox == True:

            if obstacle.obstaclePresent == False:     # stay one lane 1
                a1 = AR[k]
                b1 = BR[k]
                c1 = CR[k]
                a2 = AL[k]
                b2 = BL[k]
                c2 = CL[k]
                found_sol = True

            else:
                print("TBD")

            if found_sol == True:
                yDist[0] = (a1 * x[0] + b1 * x[1] - c1)/np.sqrt(a1**2 + b1**2)
                yDist[1] = (a2 * x[0] + b2 * x[1] - c2)/np.sqrt(a2**2 + b2**2)
                cons[0] = np.sign(yDist[0]) # not used
                cons[1] = np.sign(yDist[1]) # not used
                return yDist
            else:
                continue

        else:
            None

    if found_sol == False:
        print('No solution found in runningCons function')
        return ([np.NAN, np.NAN])


def terminalCons(u, x, t0, path, obstacle, posIdx=None):

    found_sol = False

    A = path.alongPathLines.A
    B = path.alongPathLines.B
    C = path.alongPathLines.C
    AR = path.alongPathLines.AR
    BR = path.alongPathLines.BR
    CR = path.alongPathLines.CR
    AL = path.alongPathLines.AL
    BL = path.alongPathLines.BL
    CL = path.alongPathLines.CL

    D1 = path.acrossPathLines.D1
    E1 = path.acrossPathLines.E1
    F1 = path.acrossPathLines.F1
    D2 = path.acrossPathLines.D2
    E2 = path.acrossPathLines.E2
    F2 = path.acrossPathLines.F2

    nSections = len(D1)

    if posIdx == None:
        kvec = range(nSections)
    else:
        kvec = np.arange(posIdx['number'], nSections, 1)

    for k in kvec:
        inbox = insideBox(x[0], x[1], AR[k], BR[k], CR[k], AL[k], BL[k], CL[k],
                     D1[k], E1[k], F1[k], D2[k], E2[k], F2[k])
        if inbox == True:
            if obstacle.obstaclePresent == False:     # stay one lane 1
                a = A[k]
                b = B[k]
                c = C[k]
                found_sol = True
            else:
                print("TBD")

            if found_sol == True:
                yDist = (a * x[0] + b * x[1] - c) / np.sqrt(a**2 + b**2)
                VEnd = x[2]
                return np.array([yDist]), np.array([VEnd])
            else:
                continue

        else:
            None

    if found_sol == False:
        return np.array([np.NaN]), np.array([np.NaN])


def computeOpenloopSolution(u, N, T, t0, x0):
    x = np.zeros([N, np.size(x0)])
    x[0] = x0

    for k in range(N - 1):
        u0 = u[k]
        u1 = u[k+N]
        uk = np.array([u0,u1])
        x[k+1] = system(uk, x[k], T)
    return x


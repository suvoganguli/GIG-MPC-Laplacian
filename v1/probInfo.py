from problemData import *

def system(uk, xk, T):

    if nstates == 6:
        # x0 = E, x1 = N, x2 = V, x3 = chi, x5 = Vdot, x6 = chidot
        # u0 = Vddot, u1 = chiddot

        xkp1 = [0 ,0, 0, 0, 0, 0]
        xkp1[0] = xk[0] + T * xk[2] * np.sin(xk[3])     # Edot
        xkp1[1] = xk[1] + T * xk[2] * np.cos(xk[3])     # Ndot
        xkp1[2] = xk[2] + T * xk[4]                     # Vdot
        xkp1[3] = xk[3] + T * xk[5]                     # chidot
        xkp1[4] = xk[4] + T * uk[0]                     # Vddot
        xkp1[5] = xk[5] + T * uk[1]                     # chiddot

        # xkp1 = [0 ,0, 0, 0, 0, 0]
        # xkp1[0] = xk[0] + T * V_cmd * np.sin(xk[3])     # Edot
        # xkp1[1] = xk[1] + T * V_cmd * np.cos(xk[3])     # Ndot
        # xkp1[2] = xk[2] + T * xk[4]                     # Vdot
        # xkp1[3] = xk[3] + T * xk[5]                     # chidot
        # xkp1[4] = xk[4] + T * uk[0]                     # Vddot
        # xkp1[5] = xk[5] + T * uk[1]                     # chiddot

    elif nstates == 4:
        # x0 = E, x1 = N, x2 = V, x3 = chi
        # u0 = Vdot, u1 = chidot

        xkp1 = [0, 0, 0, 0]
        xkp1[0] = xk[0] + T * xk[2] * np.sin(xk[3]) # Edot
        xkp1[1] = xk[1] + T * xk[2] * np.cos(xk[3]) # Ndot
        xkp1[2] = xk[2] + T * uk[0]                 # Vdot
        xkp1[3] = xk[3] + T * uk[1]                 # chidot

        # xkp1 = [0, 0, 0, 0]
        # xkp1[0] = xk[0] + T * V_cmd * np.sin(xk[3]) # Edot
        # xkp1[1] = xk[1] + T * V_cmd * np.cos(xk[3]) # Ndot
        # xkp1[2] = xk[2] + T * uk[0]                 # Vdot
        # xkp1[3] = xk[3] + T * uk[1]                 # chidot

    return xkp1


def runningCosts(u, x, t0, lanes, obstacle):

    A_Lane1 = lanes.lane1Lines.A_Lane
    B_Lane1 = lanes.lane1Lines.B_Lane
    C_Lane1 = lanes.lane1Lines.C_Lane
    AR_Lane1 = lanes.lane1Lines.AR_Lane
    BR_Lane1 = lanes.lane1Lines.BR_Lane
    CR_Lane1 = lanes.lane1Lines.CR_Lane
    AL_Lane1 = lanes.lane1Lines.AL_Lane
    BL_Lane1 = lanes.lane1Lines.BL_Lane
    CL_Lane1 = lanes.lane1Lines.CL_Lane

    A_Lane2 = lanes.lane2Lines.A_Lane
    B_Lane2 = lanes.lane2Lines.B_Lane
    C_Lane2 = lanes.lane2Lines.C_Lane
    AL_Lane2 = lanes.lane2Lines.AL_Lane
    BL_Lane2 = lanes.lane2Lines.BL_Lane
    CL_Lane2 = lanes.lane2Lines.CL_Lane

    D1 = lanes.acrossLines.D1
    E1 = lanes.acrossLines.E1
    F1 = lanes.acrossLines.F1
    D2 = lanes.acrossLines.D2
    E2 = lanes.acrossLines.E2
    F2 = lanes.acrossLines.F2

    npts = len(D1)
    for k in range(npts):

        cost = 0.0
        #print("Inside for-loop for k=%d" %k)

        chk1bool = False
        chk2bool = False
        chk3bool = False
        chk4bool = False

        chk1 = D1[k]*x[0] + E1[k]*x[1] - F1[k]
        chk2 = D2[k]*x[0] + E2[k]*x[1] - F2[k]
        chk3 = AR_Lane1[k]*x[0] + BR_Lane1[k]*x[1] - CR_Lane1[k]
        chk4 = AL_Lane2[k]*x[0] + BL_Lane2[k]*x[1] - CL_Lane2[k]

        myeps = 1e-3

        if chk1 >= -myeps:
            chk1bool = True
        if chk2 <= myeps:
            chk2bool = True
        if chk3 <= 0:
            chk3bool = True
        if chk4 >= 0:
            chk4bool = True

        # print([k, chk1bool, chk2bool, chk3bool, chk4bool])

        # inside road segment
        if (chk1bool) and \
            (chk2bool) and \
            (chk3bool) and \
            (chk4bool):

            #if k == 3:
            #    print("Inside box for k = %d" %k)

            if obstacle.obstaclePresent == False:
                a = A_Lane1[k]
                b = B_Lane1[k]
                c = C_Lane1[k]
            else:
                if k < obstacle.idx_StartSafeZone: # stay on lane 1
                    a = A_Lane1[k]
                    b = B_Lane1[k]
                    c = C_Lane1[k]
                elif k >= obstacle.idx_StartSafeZone and k < obstacle.idx_StartObstacle:   # start moving to lane 2
                    a = AL_Lane1[k]
                    b = BL_Lane1[k]
                    c = CL_Lane1[k]
                elif k >= obstacle.idx_StartObstacle and k < obstacle.idx_EndObstacle:  # move to lane 2
                    a = A_Lane2[k]
                    b = B_Lane2[k]
                    c = C_Lane2[k]
                elif k >= obstacle.idx_EndObstacle: # keep on lane 2
                    a = A_Lane2[k]
                    b = B_Lane2[k]
                    c = C_Lane2[k]
                else:
                    print("Error")
                    print(k)

            if nstates == 6:
                cost_p = W_P * (a * x[0] + b * x[1] - c) ** 2 / (a ** 2 + b ** 2)
                cost_v = W_V * (V_cmd - x[2])**2
                cost_vddot = W_Vddot * u[0]**2
                cost_chiddot = W_Chiddot * (u[1]*180/np.pi)**2
                cost_x = cost_p + cost_v
                cost_u = cost_vddot + cost_chiddot
                cost = cost_x + cost_u

            elif nstates == 4:
                cost_p = W_P * (a * x[0] + b * x[1] - c) ** 2 / (a ** 2 + b ** 2)
                cost_v = W_V * (V_cmd - x[2])**2
                cost_vdot = W_Vdot * u[0]**2
                cost_chidot = W_Chidot * (u[1]*180/np.pi)**2
                cost_x = cost_p + cost_v
                cost_u = cost_vdot + cost_chidot
                cost = cost_x + cost_u

            return cost

        else:
            cost = 1e6

    return cost


def runningCons(u, x, t0, lanes, obstacle):

    cons = [0,0] #[1,-1]  will lead to both constraints false in nlp.py
    yDist = np.zeros(2) # with respect to road axis (x - along, y - across)
    found_sol = False

    AR_Lane1 = lanes.lane1Lines.AR_Lane
    BR_Lane1 = lanes.lane1Lines.BR_Lane
    CR_Lane1 = lanes.lane1Lines.CR_Lane
    AL_Lane1 = lanes.lane1Lines.AL_Lane
    BL_Lane1 = lanes.lane1Lines.BL_Lane
    CL_Lane1 = lanes.lane1Lines.CL_Lane

    AR_Lane2 = lanes.lane2Lines.AR_Lane
    BR_Lane2 = lanes.lane2Lines.BR_Lane
    CR_Lane2 = lanes.lane2Lines.CR_Lane
    AL_Lane2 = lanes.lane2Lines.AL_Lane
    BL_Lane2 = lanes.lane2Lines.BL_Lane
    CL_Lane2 = lanes.lane2Lines.CL_Lane

    D1 = lanes.acrossLines.D1
    E1 = lanes.acrossLines.E1
    F1 = lanes.acrossLines.F1
    D2 = lanes.acrossLines.D2
    E2 = lanes.acrossLines.E2
    F2 = lanes.acrossLines.F2

    DO_PRINT = False
    DO_PRINT2 = False

    npts = len(D1)
    for k in range(npts):

        cost = 0.0 #DO_PRINT == True:
        if DO_PRINT == True:
            print("\n\nInside 1st for-loop, k=%d" %k)

        chk1bool = False
        chk2bool = False
        chk3bool = False
        chk4bool = False

        chk1 = D1[k]*x[0] + E1[k]*x[1] - F1[k]
        chk2 = D2[k]*x[0] + E2[k]*x[1] - F2[k]
        chk3 = AR_Lane1[k]*x[0] + BR_Lane1[k]*x[1] - CR_Lane1[k]
        chk4 = AL_Lane2[k]*x[0] + BL_Lane2[k]*x[1] - CL_Lane2[k]

        myeps = 1e-3

        if chk1 >= -myeps:
            chk1bool = True
        if chk2 <= myeps:
            chk2bool = True
        if chk3 <= myeps:
            chk3bool = True
        if chk4 >= -myeps:
            chk4bool = True

        if DO_PRINT == True:
            print(x[0], x[1])
            print(chk1, chk2, chk3, chk4)
            print(chk1bool, chk2bool, chk3bool, chk4bool)
            None


        # inside road segment
        if (chk1bool) and \
            (chk2bool) and \
            (chk3bool) and \
            (chk4bool):

            if obstacle.obstaclePresent == False:     # stay one lane 1
                a1 = AR_Lane1[k]
                b1 = BR_Lane1[k]
                c1 = CR_Lane1[k]
                a2 = AL_Lane1[k]
                b2 = BL_Lane1[k]
                c2 = CL_Lane1[k]
                found_sol = True
            else:
                if k < obstacle.idx_StartSafeZone:    # stay one lane 1
                    a1 = AR_Lane1[k]
                    b1 = BR_Lane1[k]
                    c1 = CR_Lane1[k]
                    a2 = AL_Lane1[k]
                    b2 = BL_Lane1[k]
                    c2 = CL_Lane1[k]
                    found_sol = True
                    if DO_PRINT == True:
                        print('W')
                elif k >= obstacle.idx_StartSafeZone and k < obstacle.idx_StartObstacle:   # start moving to lane 2
                    a1 = AR_Lane1[k]
                    b1 = BR_Lane1[k]
                    c1 = CR_Lane1[k]
                    a2 = AL_Lane2[k]
                    b2 = BL_Lane2[k]
                    c2 = CL_Lane2[k]
                    found_sol = True
                    if DO_PRINT == True:
                        print('G')
                elif k >= obstacle.idx_StartObstacle and k < obstacle.idx_EndObstacle:  # move to lane 2
                    a1 = AR_Lane2[k]
                    b1 = BR_Lane2[k]
                    c1 = CR_Lane2[k]
                    a2 = AL_Lane2[k]
                    b2 = BL_Lane2[k]
                    c2 = CL_Lane2[k]
                    found_sol = True
                    if DO_PRINT == True:
                        print('R')
                elif k >= obstacle.idx_EndObstacle:  # stay one lane 2
                    a1 = AR_Lane2[k]
                    b1 = BR_Lane2[k]
                    c1 = CR_Lane2[k]
                    a2 = AL_Lane2[k]
                    b2 = BL_Lane2[k]
                    c2 = CL_Lane2[k]
                    found_sol = True
                    if DO_PRINT == True:
                        print('R+')

            if found_sol == True:
                yDist[0] = (a1 * x[0] + b1 * x[1] - c1)/np.sqrt(a1**2 + b1**2)
                yDist[1] = (a2 * x[0] + b2 * x[1] - c2)/np.sqrt(a2**2 + b2**2)
                cons[0] = np.sign(yDist[0]) # not used
                cons[1] = np.sign(yDist[1]) # not used


                if DO_PRINT == True:
                    print('Found sol = %d' %(found_sol))
                    print(x[0], x[1])
                    print([a1,b1,c1])
                    print([a2,b2,c2])
                    print(yDist[0], yDist[1])
                    print(cons[0], cons[1])

                if DO_PRINT2 == True:
                    print('k = %d' %(k))
                    print(x[0:2])
                    print(F1[k]/E1[k])
                    print(F2[k]/E2[k])
                    print(cons)
                    print('returning ...')
                return yDist
            else:
                continue

        else:
            if DO_PRINT2 == True:
                print('x is Outside %d-th Road Segment' %(k))
                print(x[0:2])
                print(F1[k]/E1[k])
                print(F2[k]/E2[k])
                print(cons)
                None

    if found_sol == False:
        print('No solution found in runningCons function')
        return ([np.NAN, np.NAN])


def terminalCons(u, x, t0, lanes, obstacle):
    yDist = np.zeros(1) # with respect to road axis (x - along, y - across)
    found_sol = False

    AR_Lane1 = lanes.lane1Lines.AR_Lane
    BR_Lane1 = lanes.lane1Lines.BR_Lane
    CR_Lane1 = lanes.lane1Lines.CR_Lane
    AL_Lane1 = lanes.lane1Lines.AL_Lane
    BL_Lane1 = lanes.lane1Lines.BL_Lane
    CL_Lane1 = lanes.lane1Lines.CL_Lane

    AR_Lane2 = lanes.lane2Lines.AR_Lane
    BR_Lane2 = lanes.lane2Lines.BR_Lane
    CR_Lane2 = lanes.lane2Lines.CR_Lane
    AL_Lane2 = lanes.lane2Lines.AL_Lane
    BL_Lane2 = lanes.lane2Lines.BL_Lane
    CL_Lane2 = lanes.lane2Lines.CL_Lane

    A_Lane1 = lanes.lane1Lines.A_Lane
    B_Lane1 = lanes.lane1Lines.B_Lane
    C_Lane1 = lanes.lane1Lines.C_Lane
    A_Lane2 = lanes.lane2Lines.A_Lane
    B_Lane2 = lanes.lane2Lines.B_Lane
    C_Lane2 = lanes.lane2Lines.C_Lane


    D1 = lanes.acrossLines.D1
    E1 = lanes.acrossLines.E1
    F1 = lanes.acrossLines.F1
    D2 = lanes.acrossLines.D2
    E2 = lanes.acrossLines.E2
    F2 = lanes.acrossLines.F2

    npts = len(D1)
    for k in range(npts):
        inbox = insideBox(x[0], x[1], AR_Lane1[k], BR_Lane1[k], CR_Lane1[k], AL_Lane2[k], BL_Lane2[k], CL_Lane2[k],
                     D1[k], E1[k], F1[k], D2[k], E2[k], F2[k])
        if inbox == True:
            if obstacle.obstaclePresent == False:     # stay one lane 1
                a = A_Lane1[k]
                b = B_Lane1[k]
                c = C_Lane1[k]
                found_sol = True
            else:
                if k < obstacle.idx_StartSafeZone:  # stay one lane 1
                    a = A_Lane1[k]
                    b = B_Lane1[k]
                    c = C_Lane1[k]
                    found_sol = True
                elif k >= obstacle.idx_StartSafeZone and k < obstacle.idx_StartObstacle:   # start moving to lane 2
                    a = AR_Lane2[k]
                    b = BR_Lane2[k]
                    c = CR_Lane2[k]
                    found_sol = True
                elif k >= obstacle.idx_StartObstacle and k < obstacle.idx_EndObstacle:  # move to lane 2
                    a = A_Lane2[k]
                    b = B_Lane2[k]
                    c = C_Lane2[k]
                    found_sol = True
                elif k >= obstacle.idx_EndObstacle:  # stay on lane 2
                    a = A_Lane2[k]
                    b = B_Lane2[k]
                    c = C_Lane2[k]
                    found_sol = True

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


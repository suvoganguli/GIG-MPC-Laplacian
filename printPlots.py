import numpy as np
import probInfo
import matplotlib.pyplot as plt
import matplotlib.figure as fig
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.patches as patches
import problemData as pdata
import os
from utils import *

# Axis:
# *X, *Y = E [ft], N [ft], theta [rad] (theta is w.r.t +E axis)


def nmpcPlotSol(u_new,path,drawLPPath,x0,obstacle,pathType):

    u_mpciter = u_new.flatten(1)
    x_mpciter = probInfo.computeOpenloopSolution(u_mpciter, pdata.N, pdata.T, pdata.t0, x0)
    East = x_mpciter[:,0]
    North = x_mpciter[:,1]

    V_terminal = x_mpciter[-1,2]

    # figure 1
    plt.figure(1,figsize=(5, 7), dpi=100)

    plt.ylabel('N [ft]')
    plt.xlabel('E [ft]')
    #plt.axis('equal')

    if drawLPPath == True:

        # Detailed Path
        plt.plot(path.pathData.E, path.pathData.N, linestyle='--', color='c')

        plt.plot(path.pathData.PathStartPoint[0], path.pathData.PathStartPoint[1], marker='o', markersize=8, color='r')
        plt.plot(path.pathData.PathEndPoint[0], path.pathData.PathEndPoint[1], marker='o', markersize=8, color='g')

        if True:
            plt.plot(path.pathData.PathRightEndPointsE, path.pathData.PathRightEndPointsN,'m+')
            plt.plot(path.pathData.PathLeftEndPointsE, path.pathData.PathLeftEndPointsN,'m+')

            x1 = path.pathData.PathRightEndPointsE
            x2 = path.pathData.PathLeftEndPointsE
            y1 = path.pathData.PathRightEndPointsN
            y2 = path.pathData.PathLeftEndPointsN

            if pathType == 'default':
                plt.plot(x1, y1, 'm', x2, y2, 'm')
            else:
                #plt.plot(x1, y1, 'g', x2, y2, 'g--')
                plt.plot(x1, y1, 'm', x2, y2, 'm')

            x3 = path.pathData.PathCenterEndPointsE + pdata.delta_yRoad*np.sin(path.pathData.Theta_endpoints)
            x4 = path.pathData.PathCenterEndPointsE - pdata.delta_yRoad*np.sin(path.pathData.Theta_endpoints)
            y3 = path.pathData.PathCenterEndPointsN - pdata.delta_yRoad*np.cos(path.pathData.Theta_endpoints)
            y4 = path.pathData.PathCenterEndPointsN + pdata.delta_yRoad*np.cos(path.pathData.Theta_endpoints)

            if pathType == 'default':
                plt.plot(x3, y3, 'r', x4, y4, 'r')
            else:
                #plt.plot(x3, y3, 'k', x4, y4, 'k--')
                plt.plot(x3, y3, 'r', x4, y4, 'r')

        plt.grid(True)

        if True: # obstacle is present:

            nObs = obstacle.E.size

            if nObs > 0:
                for k in range(nObs):

                    Elb = obstacle.E[k] + obstacle.SM
                    Nlb = obstacle.N[k] + obstacle.SM
                    W = obstacle.w[k] - 2.0 * obstacle.SM
                    L = obstacle.l[k] - 2.0 * obstacle.SM

                    Theta = obstacle.Chi[k]
                    fc = "red"
                    polygon_obstacle = getPatch(Elb, Nlb, W, L, Theta, fc)


                    Elb = obstacle.E[k]
                    Nlb = obstacle.N[k]
                    W = obstacle.w[k]
                    L = obstacle.l[k]
                    Theta = obstacle.Chi[k]
                    fc = "green"
                    polygon_safezone = getPatch(Elb, Nlb, W, L, Theta, fc)

                    ax = plt.gca()
                    ax.add_patch(polygon_safezone)
                    ax.add_patch(polygon_obstacle)

    nEN = len(East)
    plt.plot(East[0:nEN], North[0:nEN], marker='x', markersize=4, color='b')
    plt.plot(East[0], North[0], marker='o', markersize=4, color='r')
    #plt.ylim([0, 240])
    #plt.xlim([-10, 40])
    plt.axis('equal')

    #plt.draw()
    plt.pause(0.01)
    #if mpciter == mpciterations-1:
        #   ax1 = f1.gca()
        #   del ax1.lines[7:12]
        #dummy = raw_input('Press Enter to continue: ')

    return V_terminal


def nmpcPlot(t,x,u,path,obstacle,tElapsed,V_terminal,latAccel,dyError,settingsFile,pathObjArray):

    f_pData = file(settingsFile, 'r')
    cols, indexToName = getColumns(f_pData, delim=" ", header=False)
    #N = np.array(cols[0]).astype(np.int)
    T = np.array(cols[1]).astype(np.float)
    ns = np.array(cols[2]).astype(np.int)
    no = np.array(cols[3]).astype(np.int)

    figno = np.zeros(9)
    figno[0] = 1

    # ncons_option is now hard-coded here since we want to create plot from
    # plotSavedData.py also.
    ncons_option = 2

    # useLatAccelCons is now hard-coded here since we want to create plot from
    # plotSavedData.py also.
    useLatAccelCons = 1

    if ns == 4:
        lb_VdotVal = np.array(cols[4]).astype(np.float)
        ub_VdotVal = np.array(cols[5]).astype(np.float)
        lb_ChidotVal = np.array(cols[6]).astype(np.float)
        ub_ChidotVal = np.array(cols[7]).astype(np.float)
        delta_yRoad = np.array(cols[8]).astype(np.float)
        lataccel_maxVal = np.array(cols[9]).astype(np.float)
        lb_V = np.array(cols[10]).astype(np.float)
        ub_V = np.array(cols[11]).astype(np.float)


    elif ns == 6:
        lb_VddotVal = np.array(cols[4]).astype(np.float)
        ub_VddotVal = np.array(cols[5]).astype(np.float)
        lb_ChiddotVal = np.array(cols[6]).astype(np.float)
        ub_ChiddotVal = np.array(cols[7]).astype(np.float)
        delta_yRoad = np.array(cols[8]).astype(np.float)
        lataccel_maxVal = np.array(cols[9]).astype(np.float)
        lb_V = np.array(cols[10]).astype(np.float)
        ub_V = np.array(cols[11]).astype(np.float)

    if ns == 4:

        # figure 2
        plt.figure(2)
        figno[1] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, x[:, [0]])  # E
        plt.ylabel('E [ft]')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(t, x[:, [1]])  # N
        plt.ylabel('N [ft]')
        plt.xlabel('t [sec]')
        plt.grid(True)


        # figure 3
        plt.figure(3)
        figno[2] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, x[:, [2]])  # V
        plt.grid(True)

        plt.ylabel('V [fps]')

        plt.subplot(212)
        plt.plot(t, u[:, [0]])  # Vdot
        plt.plot(t, lb_VdotVal*np.ones(t.shape),linestyle='--', color='r')
        plt.plot(t, ub_VdotVal*np.ones(t.shape), linestyle='--', color='r')
        plt.grid(True)

        plt.ylabel('Vdot [fps2]')
        plt.xlabel('t [sec]')
        plt.grid(True)

        print('intg(Vdot.dt) = ' + np.str(sum(abs(u[:,0]))))

        # figure 4
        plt.figure(4)
        figno[3] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, x[:, [3]]*180/np.pi)
        plt.ylabel('Chi [deg]')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(t, u[:, [1]]*180/np.pi)
        plt.plot(t, lb_ChidotVal*np.ones(t.shape)*180/np.pi,linestyle='--', color='r')
        plt.plot(t, ub_ChidotVal*np.ones(t.shape)*180/np.pi, linestyle='--', color='r')

        plt.ylabel('Chidot [deg/s]')
        plt.xlabel('t [sec]')
        plt.grid(True)

        print('intg(Chidot.dt) = ' + np.str(sum(abs(u[:, 1])*180/np.pi)))

        # figure 5
        plt.figure(6)
        figno[5] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, latAccel)
        if useLatAccelCons == 1:
            plt.plot(t, lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')
            plt.plot(t, -lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')

        plt.ylabel('Lat Accel [g]')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(t, dyError)
        plt.plot(t, delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        plt.plot(t, -delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        plt.ylabel('dy Error [m]')
        plt.xlabel('t [sec]')
        plt.grid(True)

        print('intg(latAccel.dt) = ' + np.str(sum(abs(latAccel))))

    elif ns == 6:

        # figure 2
        plt.figure(2)
        figno[1] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, x[:,[0]])  # E
        plt.ylabel('E [ft]')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(t, x[:,[1]])  # N
        plt.ylabel('N [ft]')
        plt.xlabel('t [sec]')
        plt.grid(True)


        # figure 3
        plt.figure(3)
        figno[2] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, x[:,[2]])  # V
        plt.ylabel('V [fps]')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(t, x[:,[4]])  # Vdot

        plt.ylabel('Vdot [fps2]')
        plt.xlabel('t [sec]')
        plt.grid(True)

        #print('intg(Vdot.dt) = ' + np.str(sum(abs(x[:,4]))))

        # figure 4
        plt.figure(4)
        figno[3] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, x[:,[3]]*180/np.pi)
        plt.ylabel('Chi [deg]')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(t, x[:,[5]]*180/np.pi)

        plt.ylabel('Chidot [deg/s]')
        plt.xlabel('t [sec]')
        plt.grid(True)

        #print('intg(Chidot.dt) = ' + np.str(sum(abs(x[:,5]*180/np.pi))))

        # figure 5
        plt.figure(5)
        figno[4] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, u[:,0])
        plt.plot(t, lb_VddotVal*np.ones(t.shape),linestyle='--', color='r')
        plt.plot(t, ub_VddotVal*np.ones(t.shape), linestyle='--', color='r')
        plt.ylabel('Vddot [fps3]')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(t, u[:,1]*180/np.pi)
        plt.plot(t, lb_ChiddotVal*np.ones(t.shape)*180/np.pi,linestyle='--', color='r')
        plt.plot(t, ub_ChiddotVal*np.ones(t.shape)*180/np.pi, linestyle='--', color='r')
        plt.ylabel('Chiddot [deg/s2]')
        plt.xlabel('t [sec]')
        plt.grid(True)

        print('intg(Vddot.dt) = ' + np.str(sum(abs(u[:,0]))))
        print('intg(Chiddot.dt) = ' + np.str(sum(abs(u[:,1]*180/np.pi))))

        # figure 6
        plt.figure(6)
        figno[5] = plt.gcf().number

        plt.subplot(211)
        plt.plot(t, latAccel)
        if useLatAccelCons == 1:
            plt.plot(t, lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')
            plt.plot(t, -lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')

        plt.ylabel('Lat Accel [g]')
        plt.grid(True)

        print('intg(latAccel.dt) = ' + np.str(sum(abs(latAccel))))

        plt.subplot(212)
        plt.plot(t, dyError)
        plt.plot(t, delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        plt.plot(t, -delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        plt.ylabel('dy Error [ft]')
        plt.xlabel('t [sec]')
        plt.grid(True)


    # figure 7
    iterations = np.arange(len(tElapsed))
    plt.figure(7)
    figno[6] = plt.gcf().number

    #plt.plot(iterations, tElapsed)
    plt.ylabel('CPU Time [sec]')
    plt.plot(t, tElapsed)
    #plt.xlabel('Iteration')
    plt.xlabel('t sec]')
    plt.grid(True)
    plt.xlim([0,15])
    plt.ylim([0,6])

    # figure 8
    plt.figure(8)
    figno[7] = plt.gcf().number
    plt.plot(t, V_terminal)
    if ncons_option != 3:
        plt.plot(t, lb_V * np.ones(t.shape), linestyle='--', color='r')
        plt.plot(t, ub_V * np.ones(t.shape), linestyle='--', color='r')
    plt.ylabel('V-terminal [fps]')
    plt.xlabel('time [sec]')
    plt.grid(True)

    plt.pause(0.1)

    # figure 9
    plt.figure(9,figsize=(5, 7), dpi=100)
    figno[8] = plt.gcf().number

    # Planned Path

    nPath = len(pathObjArray)

    for iPath in range(nPath):

        pathObj = pathObjArray[iPath]

        PathE = pathObj['PathE']
        PathN = pathObj['PathN']
        PathStartPoint = pathObj['PathStartPoint']
        PathEndPoint = pathObj['PathEndPoint']
        PathRightEndPointsE = pathObj['PathRightEndPointsE']
        PathRightEndPointsN = pathObj['PathRightEndPointsN']
        PathLeftEndPointsE = pathObj['PathLeftEndPointsE']
        PathLeftEndPointsN = pathObj['PathLeftEndPointsN']
        PathCenterEndPointsE = pathObj['PathCenterEndPointsE']
        PathCenterEndPointsN = pathObj['PathCenterEndPointsN']
        PathThetaEndpoints = pathObj['PathThetaEndpoints']
        PathDeltaYRoad = pathObj['PathDeltaYRoad']
        PathWidth = pathObj['PathWidth']
        ObstacleE = pathObj['ObstacleE']
        ObstacleN = pathObj['ObstacleN']
        ObstacleW = pathObj['ObstacleW']
        ObstacleL = pathObj['ObstacleL']
        ObstacleChi = pathObj['ObstacleChi']

        plt.plot(PathE, PathN, linestyle='--', color='c')

        plt.plot(PathStartPoint[0], PathStartPoint[1], marker='o', markersize=8, color='r')
        plt.plot(PathEndPoint[0], PathEndPoint[1], marker='o', markersize=8, color='g')

        if True:
            plt.plot(PathRightEndPointsE, PathRightEndPointsN,'m+')
            plt.plot(PathLeftEndPointsE, PathLeftEndPointsN,'m+')

            x1 = PathRightEndPointsE
            x2 = PathLeftEndPointsE
            y1 = PathRightEndPointsN
            y2 = PathLeftEndPointsN
            plt.plot(x1, y1, 'm', x2, y2, 'm')

            x1 = PathCenterEndPointsE - PathDeltaYRoad*np.sin(PathThetaEndpoints)
            x2 = PathCenterEndPointsE + PathDeltaYRoad*np.sin(PathThetaEndpoints)
            y1 = PathCenterEndPointsN + PathDeltaYRoad*np.cos(PathThetaEndpoints)
            y2 = PathCenterEndPointsN - PathDeltaYRoad*np.cos(PathThetaEndpoints)
            plt.plot(x1, y1, 'r', x2, y2, 'r')

        plt.grid(False)

        if True: # obstacle.Present == True:

            nObs = len(ObstacleE)
            if nObs > 0:
                for k in range(nObs):

                    Efc = ObstacleE[k] + PathWidth/2
                    Nfc = ObstacleN[k]
                    W = ObstacleW[k] - PathWidth
                    L = ObstacleL[k]
                    Theta = ObstacleChi[k]
                    fc = "red"
                    polygon_obstacle = getPatch(Efc, Nfc, W, L, Theta, fc)


                    Efc = ObstacleE[k]
                    Nfc = ObstacleN[k]
                    W = ObstacleW[k]
                    L = ObstacleL[k]
                    Theta = ObstacleChi[k]
                    fc = "green"
                    polygon_safezone = getPatch(Efc, Nfc, W, L, Theta, fc)

                    ax = plt.gca()
                    ax.add_patch(polygon_safezone)
                    ax.add_patch(polygon_obstacle)

                    None



    # Actual Path
    plt.plot(x[:,0], x[:,1], color='b')
    plt.plot(x[:,0], x[:,1], marker='o', markersize=4, color='b')
    #plt.xlim([0, 16])
    #plt.ylim([0, 128])
    plt.ylabel('N [ft]')
    plt.xlabel('E [ft]')

    # if no != 0 and T == 0.4:
    #     if no == 1:
    #         idx_LP = 658
    #         idx_EN = 23
    #     elif no == 2:
    #         idx_LP = 341
    #         idx_EN = 12
    #     if len(x[:,0]) >= idx_EN:
    #         plt.plot(PathE[idx_LP], PathN[idx_LP], marker='o', markersize=6, color='g')
    #         plt.plot(x[idx_EN,0], x[idx_EN,1], marker='o', markersize=6, color='g')
    #
    #         p1 = np.array([PathE[idx_LP],PathN[idx_LP]])
    #         p2 = np.array([x[idx_EN,0], x[idx_EN,1]])
    #         print('Cornering Distance from Laplacian Path [ft]:')
    #         print(distance(p1,p2))

    return figno

def nmpcPrint(mpciter, info, N, x, u_new, writeToFile, f, cpuTime, VTerminal):

    status = info['status']
    cost = info['obj_val']
    g = info['g']
    idx_lataccel = 2*N
    if pdata.ns == 6:
        #idx_trackingerror = 2*N + 2 # (nlp.py, option 1)
        idx_trackingerror = 2*N + 1 # (nlp.py, option 2,3)
    elif pdata.ns == 4:
        idx_trackingerror = 2*N + 1
    g1 = g[idx_lataccel]/32.2 # g
    g2 = g[idx_trackingerror] # ft
    text_g1 = "ay [g]"
    text_g2 = "dy [ft]"

    status_msg = info['status_msg']
    u = info['x']
    u0 = u[0]  # Vddot
    u1 = u[N] #Chiddot

    if pdata.ns == 6:
        text_u0 = "Vddot"
        text_u1 = "Chiddot"
    elif pdata.ns == 4:
        text_u0 = "Vdot"
        text_u1 = "Chidot"

    # 0       solved
    # 1       solved to acceptable level
    # 2       infeasible problem detected
    # 3       search direction becomes too small
    # 4       diverging iterates
    # 5       user requested stop
    # -1      maximum number of iterations exceeded
    # -2      restoration phase failed
    # -3      error in step computation
    # -10     not enough degrees of freedom
    # -11     invalid problem definition
    # -12     invalid option
    # -13     invalid number detected
    # -100    unrecoverable exception
    # -101    non-IPOPT exception thrown
    # -102    insufficient memo
    # -199    internal error

    if status == 0:
        status_msg_short = "Solved"
    elif status == 1:
        status_msg_short = "Acceptable"
    elif status == 2:
        status_msg_short = "Infeasible"
    elif status == -1:
        status_msg_short = "Max-Iter"
    elif status == 5:
        status_msg_short = "User-Stop"
    elif status == -13:
        status_msg_short = "Algorithm-Received"
    else:
        status_msg_short = status_msg[0:19]

    if writeToFile == True:
        # if mpciter == 0:
        #     f.write("%*s %*s %*s %*s %*s %*s %*s %*s %*s %*s\n" % (10, "mpciter", 10, "cost",
        #                                        7, text_u0, 7, text_u1,
        #                                        7, "V", 7, "Chi",
        #                                        7, text_g1, 7, text_g2, 15, "status_msg",
        #                                        10, "cpuTime") )

        # f.write("%*d %*.1f %*.1f %*.1f %*.1f %*.1f %*.2f %*.2f %*s %*.1f\n" % (10, mpciter, 10, cost,
        #                                          7, u0, 7, u1,
        #                                          7, x[2], 7, x[3]*180/np.pi,
        #                                          7, g1, 7, g2, 15, status_msg_short,
        #                                          10, cpuTime))

        if pdata.ns == 4:
            f.write("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %s\n" % (
                x[0], x[1], x[2], x[3],
                u0, u1,
                g1, g2,
                VTerminal, cost, cpuTime, status_msg_short ))

        elif pdata.ns == 6:
            f.write("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %s\n" % (
                x[0], x[1], x[2], x[3], x[4], x[5],
                u0, u1,
                g1, g2,
                VTerminal, cost, cpuTime, status_msg_short ))


    if mpciter == 0:
        print("%*s %*s %*s %*s %*s %*s %*s %*s %*s %*s %*s\n" % (10, "mpciter", 10, "cost",
                                               7, text_u0, 7, text_u1,
                                               7, "V", 7, "Chi", 7, "V-Terminal",
                                               7, text_g1, 7, text_g2, 15, "status_msg",
                                              10, "cpuTime") )

    print("%*d %*.1f %*.1f %*.1f %*.1f %*.1f %*.1f %*.2f %*.2f %*s %*.1f\n" % (10, mpciter, 10, cost,
                                                 7, u0, 7, u1*180/np.pi,
                                                 7, x[2], 7, x[3]*180/np.pi, 7, VTerminal,
                                                 7, g1, 7, g2, 15, status_msg_short,
                                                10, cpuTime))

    return g1, g2

def savePlots(dirname,figno):
    try:
        os.makedirs(dirname)
    except OSError:
        pass
    # let exception propagate if we just can't
    # cd into the specified directory

    oldpwd = os.getcwd()
    os.chdir(dirname)

    for k in range(len(figno)):
        plt.savefig(figno[k])

    os.chdir(oldpwd)


def plotSavedData(inFile, pathObjArray, delim, header=False):

    f = file(inFile, 'r')
    T = np.array(inFile[14]).astype(np.int)
    ns = np.array(inFile[18]).astype(np.int)
    cols, indexToName = getColumns(f, delim=delim, header=header)

    if ns == 4:
        nt = len(cols[0])
        t = np.float(T) * np.arange(0, nt)/10

        x = np.zeros((4, nt))/10
        x[0] = np.array(cols[0]).astype(np.float)
        x[1] = np.array(cols[1]).astype(np.float)
        x[2] = np.array(cols[2]).astype(np.float)
        x[3] = np.array(cols[3]).astype(np.float)

        u = np.zeros((2, nt))
        u[0] = np.array(cols[4]).astype(np.float)
        u[1] = np.array(cols[5]).astype(np.float)

        path = None
        obstacle = None

        latAccel = np.array(cols[6]).astype(np.float)
        dyError = np.array(cols[7]).astype(np.float)
        VTerminal = np.array(cols[8]).astype(np.float)

        cpuTime = np.array(cols[10]).astype(np.float)


    elif ns == 6:
        nt = len(cols[0])

        t = np.float(T) * np.arange(0, nt)/10

        x = np.zeros((6, nt))
        x[0] = np.array(cols[0]).astype(np.float)
        x[1] = np.array(cols[1]).astype(np.float)
        x[2] = np.array(cols[2]).astype(np.float)
        x[3] = np.array(cols[3]).astype(np.float)
        x[4] = np.array(cols[4]).astype(np.float)
        x[5] = np.array(cols[5]).astype(np.float)

        u = np.zeros((2, nt))
        u[0] = np.array(cols[6]).astype(np.float)
        u[1] = np.array(cols[7]).astype(np.float)

        path = None
        obstacle = None

        latAccel = np.array(cols[8]).astype(np.float)
        dyError = np.array(cols[9]).astype(np.float)
        VTerminal = np.array(cols[10]).astype(np.float)

        cpuTime = np.array(cols[12]).astype(np.float)


    suffix = inFile[7:]
    settingsFile = 'settings' + suffix
    nmpcPlot(t, x.T, u.T, path, obstacle, cpuTime, VTerminal, latAccel, dyError, settingsFile, pathObjArray)

    f.close()

    plt.pause(0.1)

    return np.mean(cpuTime)




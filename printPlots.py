import numpy as np
from probInfo import *
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


def nmpcPlotSol(u_new,path,mpciter,x0,obstacle,case):

    u_mpciter = u_new.flatten(1)
    x_mpciter = computeOpenloopSolution(u_mpciter, N, T, t0, x0)
    East = x_mpciter[:,0]
    North = x_mpciter[:,1]

    V_terminal = x_mpciter[-1,2]

    # figure 1
    f1 = plt.figure(1,figsize=(5, 7), dpi=100)
    plt.ylabel('N [ft]')
    plt.xlabel('E [ft]')
    #plt.axis('equal')

    if mpciter == 0:

        plt.figure(f1.number)

        # Detailed Path
        plt.plot(path.pathData.E, path.pathData.N, linestyle='--', color='c')

        # Laplacian Path
        #plt.plot(path.pathData.pathLaplacian[0,:], path.pathData.pathLaplacian[1,:], linestyle='--', color='k')

        #plt.plot(path.pathData.PathLeftBoundaryE, path.pathData.PathLeftBoundaryN, linestyle='-', color='k')
        #plt.plot(path.pathData.PathRightBoundaryE, path.pathData.PathRightBoundaryN, linestyle='-', color='k')

        plt.plot(path.pathData.PathStartPoint[0], path.pathData.PathStartPoint[1], marker='o', markersize=8, color='r')
        plt.plot(path.pathData.PathEndPoint[0], path.pathData.PathEndPoint[1], marker='o', markersize=8, color='g')

        if True:
            plt.plot(path.pathData.PathRightEndPointsE, path.pathData.PathRightEndPointsN,'m+')
            plt.plot(path.pathData.PathLeftEndPointsE, path.pathData.PathLeftEndPointsN,'m+')

            x1 = path.pathData.PathRightEndPointsE
            x2 = path.pathData.PathLeftEndPointsE
            y1 = path.pathData.PathRightEndPointsN
            y2 = path.pathData.PathLeftEndPointsN
            plt.plot(x1, y1, 'm', x2, y2, 'm')

            x1 = path.pathData.PathCenterEndPointsE - pdata.delta_yRoad*np.sin(path.pathData.Theta_endpoints)
            x2 = path.pathData.PathCenterEndPointsE + pdata.delta_yRoad*np.sin(path.pathData.Theta_endpoints)
            y1 = path.pathData.PathCenterEndPointsN + pdata.delta_yRoad*np.cos(path.pathData.Theta_endpoints)
            y2 = path.pathData.PathCenterEndPointsN - pdata.delta_yRoad*np.cos(path.pathData.Theta_endpoints)
            plt.plot(x1, y1, 'r', x2, y2, 'r')

            #for i in range(len(path.pathData.LaneRightEndPointsX)):
            #    x1 = path.pathData.LaneRightEndPointsX[i]
            #    y1 = path.pathData.LaneRightEndPointsY[i]
            #    x2 = path.pathData.LaneLeftEndPointsX[i]
            #    y2 = path.pathData.LaneLeftEndPointsY[i]
            #    plt.plot([x1, x2], [y1, y2], 'm')

        ax1 = f1.gca()
        ax1.grid(True)

        if True: # obstacle.Present == True:

            nObs = len(obstacle.E)
            if nObs > 0:
                for k in range(nObs):

                    Efc = obstacle.E[k] + pathWidth/2
                    Nfc = obstacle.N[k]
                    W = obstacle.w[k] - pathWidth
                    L = obstacle.l[k]
                    Theta = obstacle.Chi[k]
                    fc = "red"
                    polygon_obstacle = getPatch(Efc, Nfc, W, L, Theta, fc)


                    Efc = obstacle.E[k]
                    Nfc = obstacle.N[k]
                    W = obstacle.w[k]
                    L = obstacle.l[k]
                    Theta = obstacle.Chi[k]
                    fc = "green"
                    polygon_safezone = getPatch(Efc, Nfc, W, L, Theta, fc)

                    ax1.add_patch(polygon_safezone)
                    ax1.add_patch(polygon_obstacle)

    plt.figure(f1.number)
    nEN = len(East)
    plt.plot(East[0:nEN], North[0:nEN], marker='x', markersize=4, color='b')
    plt.plot(East[0], North[0], marker='o', markersize=4, color='r')
    plt.xlim([0, 16])
    plt.ylim([0, 128])
    #ax1.set_xlim([0, 16])
    #ax1.set_ylim([0, 128])


    plt.pause(0.01)
    #if mpciter < mpciterations-1:
    #   ax1 = f1.gca()
    #   del ax1.lines[7:12]

    return V_terminal


def nmpcPlot(t,x,u,path,obstacle,tElapsed,V_terminal,latAccel,dyError):


    if ns == 6:

        figno = np.zeros(7)

        # figure 2
        f, ax = plt.subplots(2)
        figno[0] =  plt.gcf().number
        ax[0].plot(t, x[:,[0]])  # E
        ax[1].plot(t, x[:,[1]])  # N
        ax[0].set_ylabel('E [ft]')
        ax[1].set_ylabel('N [ft]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 3
        f, ax = plt.subplots(2)
        figno[1] = plt.gcf().number
        ax[0].plot(t, x[:,[2]])  # V

        if ns_option != 3:
            ax[0].plot(t, lb_V*np.ones(t.shape),linestyle='--', color='g')
            ax[0].plot(t, ub_V*np.ones(t.shape), linestyle='--', color='g')

        ax[1].plot(t, x[:,[4]])  # Vdot
        ax[0].set_ylabel('V [fps]')
        ax[1].set_ylabel('Vdot [fps2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 4
        f, ax = plt.subplots(2)
        figno[2] = plt.gcf().number
        ax[0].plot(t, x[:,[3]]*180/np.pi)
        ax[1].plot(t, x[:,[5]]*180/np.pi)
        ax[0].set_ylabel('Chi [deg]')
        ax[1].set_ylabel('Chidot [deg/s]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 5
        f, ax = plt.subplots(2)
        figno[3] = plt.gcf().number
        ax[0].plot(t, u[:,0])
        ax[0].plot(t, lb_VddotVal*np.ones(t.shape),linestyle='--', color='r')
        ax[0].plot(t, ub_VddotVal*np.ones(t.shape), linestyle='--', color='r')

        ax[1].plot(t, u[:,1]*180/np.pi)
        ax[1].plot(t, lb_ChiddotVal*np.ones(t.shape)*180/np.pi,linestyle='--', color='r')
        ax[1].plot(t, ub_ChiddotVal*np.ones(t.shape)*180/np.pi, linestyle='--', color='r')

        ax[0].set_ylabel('Vddot [fps3]')
        ax[1].set_ylabel('Chiddot [deg/s2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 6
        f, ax = plt.subplots(2)
        figno[3] = plt.gcf().number
        # ax[0].plot(t, x[:, [2]] * u[:, [1]] / 32.2)  # V*Chidot
        ax[0].plot(t, latAccel)
        if useLatAccelCons == 1:
            ax[0].plot(t, lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')
            ax[0].plot(t, -lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')

        ax[0].set_ylabel('Lat Accel [g]')
        ax[0].grid(True)

        ax[1].plot(t, dyError)
        ax[1].plot(t, delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        ax[1].plot(t, -delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        ax[1].set_ylabel('dy Error [m]')
        ax[1].set_xlabel('t [sec]')
        ax[1].grid(True)


    elif ns == 4:

        figno = np.zeros(7)

        # figure 2
        f, ax = plt.subplots(2)
        figno[0] = plt.gcf().number
        ax[0].plot(t, x[:, [0]])  # E
        ax[1].plot(t, x[:, [1]])  # N
        ax[0].set_ylabel('E [ft]')
        ax[1].set_ylabel('N [ft]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 3
        f, ax = plt.subplots(2)
        figno[1] = plt.gcf().number
        ax[0].plot(t, x[:, [2]])  # V

        if ns_option != 3:
            ax[0].plot(t, lb_V*np.ones(t.shape),linestyle='--', color='g')
            ax[0].plot(t, ub_V*np.ones(t.shape), linestyle='--', color='g')

        ax[0].set_ylabel('V [fps]')

        ax[1].plot(t, u[:, [0]])  # Vdot
        ax[1].plot(t, lb_VdotVal*np.ones(t.shape),linestyle='--', color='r')
        ax[1].plot(t, ub_VdotVal*np.ones(t.shape), linestyle='--', color='r')

        ax[1].set_ylabel('Vdot [fps2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 4
        f, ax = plt.subplots(2)
        figno[2] = plt.gcf().number
        ax[0].plot(t, x[:, [3]]*180/np.pi)

        ax[1].plot(t, u[:, [1]]*180/np.pi)
        ax[1].plot(t, lb_ChidotVal*np.ones(t.shape)*180/np.pi,linestyle='--', color='r')
        ax[1].plot(t, ub_ChidotVal*np.ones(t.shape)*180/np.pi, linestyle='--', color='r')

        ax[0].set_ylabel('Chi [deg]')
        ax[1].set_ylabel('Chidot [deg/s]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)


        # figure 5
        f, ax = plt.subplots(2)
        figno[3] = plt.gcf().number
        # ax[0].plot(t, x[:, [2]] * u[:, [1]] / 32.2)  # V*Chidot
        ax[0].plot(t, latAccel)
        if useLatAccelCons == 1:
            ax[0].plot(t, lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')
            ax[0].plot(t, -lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')

        ax[0].set_ylabel('Lat Accel [g]')
        ax[0].grid(True)

        ax[1].plot(t, dyError)
        ax[1].plot(t, delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        ax[1].plot(t, -delta_yRoad * np.ones(t.shape), linestyle='--', color='r')
        ax[1].set_ylabel('dy Error [m]')
        ax[1].set_xlabel('t [sec]')
        ax[1].grid(True)

    # figure 6/7
    iterations = np.arange(len(tElapsed))
    f, ax = plt.subplots(1)
    figno[5] = plt.gcf().number
    plt.plot(iterations, tElapsed)
    ax.set_ylabel('CPU Time [sec]')
    ax.set_xlabel('Iteration')
    ax.grid(True)


    # figure 7/8
    f, ax = plt.subplots(1)
    figno[6] = plt.gcf().number
    plt.plot(t, V_terminal)
    if ns_option != 3:
        ax.plot(t, lb_V * np.ones(t.shape), linestyle='--', color='r')
        ax.plot(t, ub_V * np.ones(t.shape), linestyle='--', color='r')
    plt.ylabel('V-terminal [fps]')
    plt.xlabel('time [sec]')
    ax.grid(True)

    plt.show()

    return figno

def nmpcPrint(mpciter, info, N, x, u_new, writeToFile, f, cpuTime, VTerminal):

    status = info['status']
    cost = info['obj_val']
    g = info['g']
    idx_lataccel = 2*N
    if ns == 6:
        #idx_trackingerror = 2*N + 2 # (nlp.py, option 1)
        idx_trackingerror = 2*N + 1 # (nlp.py, option 2,3)
    elif ns == 4:
        idx_trackingerror = 2*N + 1
    g1 = g[idx_lataccel]/32.2 # g
    g2 = g[idx_trackingerror] # ft
    text_g1 = "ay [g]"
    text_g2 = "dy [ft]"

    status_msg = info['status_msg']
    u = info['x']
    u0 = u[0]  # Vddot
    u1 = u[N] #Chiddot

    if ns == 6:
        text_u0 = "Vddot"
        text_u1 = "Chiddot"
    elif ns == 4:
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

        if ns == 4:
            f.write("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %s\n" % (
                x[0], x[1], x[2], x[3],
                u0, u1,
                g1, g2,
                VTerminal, cost, cpuTime, status_msg_short ))

        elif ns == 6:
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


def plotSavedData(inFile, delim, header=True):

    f = file(inFile, 'r')
    cols, indexToName = getColumns(f, delim=delim, header=header)

    if ns == 4:
        nt = len(cols[0])

        t = T * np.arange(0, nt)

        x = np.zeros((4, nt))
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

        t = T * np.arange(0, nt)

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


    nmpcPlot(t, x.T, u.T, path, obstacle, cpuTime, VTerminal, latAccel, dyError)

    f.close()

    None

    return cols, indexToName





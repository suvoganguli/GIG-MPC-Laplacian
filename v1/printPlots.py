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

    # figure 1
    f1 = plt.figure(1,figsize=(6, 8), dpi=100)
    plt.ylabel('N [ft]')
    plt.xlabel('E [ft]')
    print('')
    None

    if mpciter == 0:
        plt.plot(path.X, path.Y, linestyle='-', color='k')
        ax1 = f1.gca()
        ax1.grid(True)

        if obstacle.obstaclePresent == True:
            Efc = obstacle.EObsStartSafeZone
            Nfc = obstacle.NObsStartSafeZone
            W = obstacle.widthSafeZone
            L = obstacle.lengthSafeZone
            Theta = obstacle.thetaSafeZone
            fc = "green"
            polygon_safezone = getPatch(Efc, Nfc, W, L, Theta, fc)

            Efc = obstacle.EObsStart
            Nfc = obstacle.NObsStart
            W = obstacle.widthObstacle
            L = obstacle.lengthObstacle
            Theta = obstacle.thetaObstacle
            fc = "red"
            polygon_obstacle = getPatch(Efc, Nfc, W, L, Theta, fc)

            ax1.add_patch(polygon_safezone)
            ax1.add_patch(polygon_obstacle)

    plt.figure(f1.number)
    nEN = len(East)
    plt.plot(East[0:nEN], North[0:nEN], marker='x', markersize=4, color='b')
    plt.plot(East[0], North[0], marker='o', markersize=4, color='r')
    #ax1.set_xlim([-100, 10])
    #ax1.set_ylim([200, 425])

    plt.pause(0.01)
    #if mpciter < mpciterations-1:
    #   ax1 = f1.gca()
    #   del ax1.lines[7:12]


def nmpcPlot(t,x,u,path,obstacle,tElapsed,case):


    if nstates == 6:

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
        ax[1].plot(t, u[:,1]*180/np.pi)
        ax[0].set_ylabel('Vddot [fps3]')
        ax[1].set_ylabel('Chiddot [deg/s2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 6
        f, ax = plt.subplots(1)
        figno[4] = plt.gcf().number
        plt.plot(t, x[:,[2]] * x[:,[5]] / 32.2)  # V*Chidot
        ax.set_ylabel('Lat Accel [g]')
        ax.set_xlabel('t [sec]')
        ax.grid(True)

        # figure 7
        figno[5] = plt.gcf().number
        f, ax = plt.subplots(1, figsize=(6, 8), dpi=100)  #sharex=True
        lw = 1.0
        ax.plot(path.X, path.Y, linewidth = lw, linestyle='--', color='k')
        ax.plot(x[:,0],x[:,1], linestyle='-', color='b')
        ax.set_ylabel('N [ft]')
        ax.set_xlabel('E [ft]')
        ax.grid(True)
        #plt.axis('equal')

    elif nstates == 4:

        figno = np.zeros(6)

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
        ax[1].plot(t, u[:, [0]])  # Vdot
        ax[0].set_ylabel('V [fps]')
        ax[1].set_ylabel('Vdot [fps2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 4
        f, ax = plt.subplots(2)
        figno[2] = plt.gcf().number
        ax[0].plot(t, x[:, [3]] * 180 / np.pi)
        ax[1].plot(t, u[:, [1]] * 180 / np.pi)
        ax[0].set_ylabel('Chi [deg]')
        ax[1].set_ylabel('Chidot [deg/s]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)


        # figure 5
        f, ax = plt.subplots(1)
        figno[3] = plt.gcf().number
        plt.plot(t, x[:, [2]] * u[:, [1]] / 32.2)  # V*Chidot
        ax.set_ylabel('Lat Accel [g]')
        ax.set_xlabel('t [sec]')
        ax.grid(True)

        # figure 6
        figno[4] = plt.gcf().number
        f, ax = plt.subplots(1, figsize=(6, 8), dpi=100)  # sharex=True
        lw = 1.0
        ax.plot(path.X, path.Y, linewidth=lw, linestyle='-', color='k')
        ax.plot(x[:, 0], x[:, 1], linestyle='-', color='b')
        ax.set_ylabel('N [ft]')
        ax.set_xlabel('E [ft]')
        ax.grid(True)
        # plt.axis('equal')

    if case == 'UGVDemo1' or case == 'UGVDemo1Debug':
        #ax.set_xlim([-150, 50])
        #ax.set_ylim([300,500])
        None
    else:
        #xlimval = obstacle.xlimval
        #ylimval = obstacle.ylimval
        #plt.xlim(xlimval[0], xlimval[1])
        #plt.ylim(ylimval[0],ylimval[1])
        None

    if obstacle.obstaclePresent == True:
        xfc = obstacle.EObsStartSafeZone
        yfc = obstacle.NObsStartSafeZone
        W = obstacle.widthSafeZone
        L = obstacle.lengthSafeZone
        Theta = obstacle.thetaSafeZone
        fc = "green"
        polygon_safezone = getPatch(xfc, yfc, W, L, Theta, fc)

        xfc = obstacle.EObsStart
        yfc = obstacle.NObsStart
        W = obstacle.widthObstacle
        L = obstacle.lengthObstacle
        Theta = obstacle.thetaObstacle
        fc = "red"
        polygon_obstacle = getPatch(xfc, yfc, W, L, Theta, fc)

        ax.add_patch(polygon_safezone)
        ax.add_patch(polygon_obstacle)



    # figure 7
    iterations = np.arange(len(tElapsed))
    f, ax = plt.subplots(1)
    figno[5] = plt.gcf().number
    plt.plot(iterations, tElapsed)
    ax.set_ylabel('CPU Time [sec]')
    ax.set_xlabel('Iteration')
    ax.grid(True)

    plt.show()

    return figno

def nmpcPrint(mpciter, info, N, x, writeToFile, f, t):

    status = info['status']
    cost = info['obj_val']
    g = info['g']
    idx_lataccel = 2*N
    if nstates == 6:
        idx_trackingerror = 2*N+2
    elif nstates == 4:
        idx_trackingerror = 2 * N + 1
    g1 = g[idx_lataccel]/32.2 # g
    g2 = g[idx_trackingerror] # ft
    text_g1 = "ay [g]"
    text_g2 = "dy [ft]"

    status_msg = info['status_msg']
    u = info['x']
    u0 = u[0]  # Vddot
    u1 = u[N]*180/np.pi  #Chiddot

    if nstates == 6:
        text_u0 = "Vddot"
        text_u1 = "Chiddot"
    elif nstates == 4:
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
        status_msg_short = "Max Iter"
    elif status == 5:
        status_msg_short = "User Stop"
    else:
        status_msg_short = status_msg[0:19]

    if writeToFile == True:
        if mpciter == 0:
            f.write("%*s %*s %*s %*s %*s %*s %*s %*s %*s %*s\n" % (10, "mpciter", 10, "cost",
                                               7, text_u0, 7, text_u1,
                                               7, "V", 7, "Chi",
                                               7, text_g1, 7, text_g2, 15, "status_msg",
                                               10, "cpuTime") )

        f.write("%*d %*.1f %*.1f %*.1f %*.1f %*.1f %*.2f %*.2f %*s %*.1f\n" % (10, mpciter, 10, cost,
                                                 7, u0, 7, u1,
                                                 7, x[2], 7, x[3]*180/np.pi,
                                                 7, g1, 7, g2, 15, status_msg_short,
                                                 10, t))

    if mpciter == 0:
        print("%*s %*s %*s %*s %*s %*s %*s %*s %*s %*s\n" % (10, "mpciter", 10, "cost",
                                               7, text_u0, 7, text_u1,
                                               7, "V", 7, "Chi",
                                               7, text_g1, 7, text_g2, 15, "status_msg",
                                              10, "cpuTime") )

    print("%*d %*.1f %*.1f %*.1f %*.1f %*.1f %*.2f %*.2f %*s %*.2f\n" % (10, mpciter, 10, cost,
                                                 7, u0, 7, u1,
                                                 7, x[2], 7, x[3]*180/np.pi,
                                                 7, g1, 7, g2, 15, status_msg_short,
                                                10,t))

    None

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

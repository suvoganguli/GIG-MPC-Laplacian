from path import *
from nmpc import *
from obstacleData import *
import printPlots
import time, datetime
import shutil, distutils.dir_util

# -------------------------------------------------------------------
# Main.py lets the user run different test cases for Model
# Predictive Control (MPC) based trajectory generation
#
# The user needs to select 3 items:
# 1. Test Case Type: Road shape and orientation
# 2. Experiment Number: Sets various design parameters for MPC
# 3. Number of States: Sets the vehicles states and control states
#
# Edit problemData.py to run different experiments
#
# 01/25/2018
# -------------------------------------------------------------------

# Path data
pathClass = pathInfo('default')
path = pathClass()

# Obstacle data (static)
obstacleClass = obstacleInfo(obstaclePresent, obstacleE, obstacleN, obstacleChi, obstacleWidth, obstacleLength)
obstacle = obstacleClass()

# Storage data
t = np.zeros([mpciterations, 1])
x = np.zeros([mpciterations, nx])
u = np.zeros([mpciterations, nu])

# Iteration data
x0[3] = np.pi/2 - path.pathData.Theta[0]  # align vehicle heading with road heading
tmeasure = t0
xmeasure = x0
u_new = np.zeros([1,nu])
mpciter = 0

# print to File
writeToFile = True
if writeToFile == True:
    fileName = 'logFile.txt'
    fHandle = open(fileName, 'w')
else:
    fHandle = -1
    fileName = ''

saveData = True

tElapsed = np.zeros(mpciterations)

posIdx = getPosIdx(x0[0], x0[1], path, posIdx0)


# Main loop
while mpciter < mpciterations:

    #if mpciter % 25 == 0:
    #    print(" k = %d" % mpciter)

    #  get new initial value
    t0, x0 = measureInitialValue(tmeasure, xmeasure)

    # search for obstacle
    detected = detectObstacle(x0, detectionWindow, obstacle)
    if runOnce == True:
        detected = True
        runOnce = False
    else:
        detected = False

    # create new path, if necessary
    if detected == True:
        pathClass = pathInfo('newpath', obstacle)
        path = pathClass()
        x0[3] = np.pi/2 - path.pathData.Theta[0]  # align vehicle heading with road heading

    # solve optimal control problem
    tStart = time.time()
    u_new, info = solveOptimalControlProblem(N, t0, x0, u0, T, ncons, nu, path, obstacle, posIdx)
    tElapsed[mpciter] = (time.time() - tStart)

    # solution information
    printPlots.nmpcPrint(mpciter, info, N, x0, u_new, writeToFile, fHandle, tElapsed[mpciter])

    # mpc  future path plot
    printPlots.nmpcPlotSol(u_new, path, mpciter, x0, obstacle, None)

    # store closed loop data
    t[mpciter] = tmeasure
    for k in range(nx):
        x[mpciter, k] = xmeasure[k]
    for j in range(nu):
        u[mpciter, j] = u_new[0,j]

    # apply control
    tmeasure, xmeasure = applyControl(T, t0, x0, u_new)

    # prepare restart
    u0 = shiftHorizon(N, u_new)

    posIdx = getPosIdx(xmeasure[0], xmeasure[1], path, posIdx)

    mpciter = mpciter + 1

# close log file
if writeToFile == True:
    fHandle.close()

if saveData == True:

    rundate = datetime.datetime.now().strftime("%Y-%m-%d")

    rundir = './run_' + rundate + '/'
    distutils.dir_util.mkpath(rundir)

    suffix = '_N' + str(N) + '_Tp' + str(int(10*T)) + '_ns' + str(ns) + '_no' + str(no)
    dst_file = rundir + 'logFile' + suffix + '.txt'
    shutil.copyfile('logFile.txt', dst_file)

    dst_fig = rundir + 'path' + suffix + '.png'
    fig = plt.figure(1)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    print('saved data and figure')

# create plots
figno = printPlots.nmpcPlot(t, x, u, path, obstacle, tElapsed, None)
print('done!')

# Save Data
# answer =  raw_input('Save Figures and Data [y/n]:  ')
# if answer == 'y':
#     dirname = raw_input('Enter Folder Name: ')
#     printPlots.savePlots(dirname, figno)



# -------------------------------------------------------------------


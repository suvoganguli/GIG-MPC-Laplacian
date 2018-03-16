from path import *
from nmpc import *
from obstacleData import *
import problemData as pdata
import printPlots
import time, datetime
import shutil, distutils.dir_util
import os

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
pathClass = pathInfo('default', startPoint, endPoint)
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

# Print to File
writeToFile = True
if writeToFile == True:
    fileName = 'logFile.txt'
    fHandle = open(fileName, 'w')
else:
    fHandle = -1
    fileName = ''

# Initialize storage arrays
tElapsed = np.zeros(mpciterations)
VTerminal = np.zeros(mpciterations)
latAccel = np.zeros(mpciterations)
dyError = np.zeros(mpciterations)

# Speficy initial position index
posIdx = getPosIdx(x0[0], x0[1], path, posIdx0)

# Specify Booleans
saveData = True
plotData = True
drawLPPath = True
runIfDetected = True

# Create array of paths
pathObj = makePathObj(pdata, path, obstacle)
pathObjArray = [pathObj]

# Main loop
while mpciter < mpciterations:

    #  get new initial value
    t0, x0 = measureInitialValue(tmeasure, xmeasure)

    # search for obstacle
    detected = detectObstacle(x0, detectionWindow, obstacle)

    # create new path (only once), if obstacle detected
    if detected == True and runIfDetected == True:
        d = 5.0
        chi = np.pi/2 - path.pathData.Theta[0]
        dN = d*np.cos(chi)
        dE = d*np.sin(chi)
        startPoint = np.array([x0[0]-dE, x0[1]-dN])

        pathClass = pathInfo('newpath', startPoint, endPoint, obstacle)
        path = pathClass()

        pathObj = makePathObj(pdata, path, obstacle)
        pathObjArray.append(pathObj)

        x0[3] = chi  # align vehicle heading with road heading
        runIfDetected = False
        drawLPPath = True

    # solve optimal control problem
    tStart = time.time()
    u_new, info = solveOptimalControlProblem(N, t0, x0, u0, T, ncons, nu, path, obstacle, posIdx, ncons_option)
    tElapsed[mpciter] = (time.time() - tStart)

    # mpc  future path plot
    VTerminal[mpciter] = printPlots.nmpcPlotSol(u_new, path, drawLPPath, x0, obstacle)
    drawLPPath = False

    # solution information
    latAccel[mpciter], dyError[mpciter] = printPlots.nmpcPrint(mpciter, info, N, x0, u_new, writeToFile,
                                                               fHandle, tElapsed[mpciter], VTerminal[mpciter])

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


rundate = datetime.datetime.now().strftime("%Y-%m-%d")
rundir = './run_' + rundate + '/'
if N < 10:
    suffix = '_N0' + str(N) + '_Tp' + str(int(10*T)) + '_ns' + str(ns) + '_no' + str(no)
else:
    suffix = '_N' + str(N) + '_Tp' + str(int(10 * T)) + '_ns' + str(ns) + '_no' + str(no)

suffix = suffix + '_Popup'

if saveData == True:

    distutils.dir_util.mkpath(rundir)
    dst_file = rundir + 'logFile' + suffix + '.txt'
    shutil.copyfile('logFile.txt', dst_file)

    # figure 1: path
    dst_fig = rundir + 'path' + suffix + '_Before.png'
    fig = plt.figure(1)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    file_pkl = rundir + 'pathDict_no' + str(no) + '_Popup' + '.pkl'
    savepkl(pathObjArray, file_pkl)

    print('saved data and figure')


# create plots
oldpwd = os.getcwd()
os.chdir(rundir)
settingsFile = 'settings' + suffix + '.txt'
figno = printPlots.nmpcPlot(t, x, u, path, obstacle, tElapsed, VTerminal, latAccel, dyError, settingsFile, pathObjArray)
os.chdir(oldpwd)

if saveData == True:

    # figure 2: E, N
    dst_fig = rundir + 'E-N' + suffix + '.png'
    fig = plt.figure(2)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 3: V, Vdot
    dst_fig = rundir + 'V-Vdot' + suffix + '.png'
    fig = plt.figure(3)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 4: Chi, Chidot
    dst_fig = rundir + 'Chi-Chidot' + suffix + '.png'
    fig = plt.figure(4)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 5: LatAccel, dy
    dst_fig = rundir + 'LatAccel-dy' + suffix + '.png'
    fig = plt.figure(6)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    if ns == 6:
        # figure 6: V, Vdot
        dst_fig = rundir + 'Vddot-Chiddot' + suffix + '.png'
        fig = plt.figure(5)
        plt.pause(0.01)
        fig.savefig(dst_fig)

    # figure 7: CPU time
    dst_fig = rundir + 'CPUtime' + suffix + '.png'
    fig = plt.figure(7)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 8: V-terminal
    dst_fig = rundir + 'V-terminal' + suffix + '.png'
    fig = plt.figure(8)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 9: path
    dst_fig = rundir + 'path' + suffix + '_After.png'
    fig = plt.figure(9)
    plt.pause(0.01)
    fig.savefig(dst_fig)


print('done!')
plt.show()


# Save Data
# answer =  raw_input('Save Figures and Data [y/n]:  ')
# if answer == 'y':
#     dirname = raw_input('Enter Folder Name: ')
#     printPlots.savePlots(dirname, figno)



# -------------------------------------------------------------------


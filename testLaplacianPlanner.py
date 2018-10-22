import numpy as np
from laplacianPlanner import *
from obstacleData import *
import time
import matplotlib.pyplot as plt
import matplotlib.figure as fig
import matplotlib.patches as patches

debug = False

# -----------------------------------------------------------------
# Create Grid

scaleFactorE = 1
scaleFactorN = 1
scaleFactorh = 1

widthSpace = 32 # ft
lengthSpace = 128  # ft
heightSpace = 8 # ft

# ---------------------------------------------------------------------
# debug

if debug:
    scaleFactorE = 2
    scaleFactorN = 2
    scaleFactorh = 1

    widthSpace = 32 # ft
    lengthSpace = 128  # ft
    heightSpace = 8 # ft

# ---------------------------------------------------------------------

widthSpace = int(widthSpace * scaleFactorE) # ft
lengthSpace = int(lengthSpace * scaleFactorN)  # ft
heightSpace = int(heightSpace * scaleFactorh) # ft

gridSize = 1 # ft/unit
gridClass = createGrid(gridSize, lengthSpace, widthSpace, heightSpace)
grid = gridClass()

# ---------------------------------------------------------------------
# Path Data

nE = grid.nE
nN = grid.nN
nU = grid.nU
nU_low = grid.nU_low
height = grid.height  # ft

# Start and End Points
startPoint = np.array([16 * scaleFactorE, 4 * scaleFactorN])  # E (ft), N (ft)
endPoint = np.array([16 * scaleFactorE, 124 * scaleFactorN])  # E (ft), N (ft)

# ---------------------------------------------------------------------
# debug

if debug:
    startPoint = np.array([7.5 * scaleFactorE, 1 * scaleFactorN])  # E (ft), N (ft)
    endPoint = np.array([7.5 * scaleFactorE, 115 * scaleFactorN])  # E (ft), N (ft)

# ---------------------------------------------------------------------

startPoint_ = np.append(startPoint, height)
endPoint_ = np.append(endPoint, height)

# ---------------------------------------------------------------------
# Obstacle Data

obstaclePresent = True
safetyMargin = 2

obstacleType = 'small'

if obstaclePresent == False:
    obstacleN = np.array([]) * scaleFactorN  # ft, left-bottom
    obstacleChi = np.array([])  # rad
    obstacleLength = np.array([]) * scaleFactorN  # ft
    obstacleWidth = np.array([]) * scaleFactorE + 0.01*0
    obstacleE = []  # ft, left-bottom

elif obstaclePresent == True:

    if obstacleType == 'small':
        obstacleN = np.array([54.0]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0])  # rad
        obstacleLength = np.array([6]) * scaleFactorN  # ft
        obstacleWidth = np.array([6]) * scaleFactorE + 0.01
        obstacleE = startPoint[0] - obstacleWidth / 2  # ft, left-bottom

    if obstacleType == 'medium':
        obstacleN = np.array([54.0]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0])  # rad
        obstacleLength = np.array([12]) * scaleFactorN  # ft
        obstacleWidth = np.array([12]) * scaleFactorE + 0.01
        obstacleE = startPoint[0] - obstacleWidth / 2  # ft, left-bottom


    elif obstacleType == 'large':
        obstacleN = np.array([54.0]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0])  # rad
        obstacleLength = np.array([24]) * scaleFactorN  # ft
        obstacleWidth = np.array([24]) * scaleFactorE + 0.01
        obstacleE = startPoint[0] - obstacleWidth / 2 # ft, left-bottom


if debug:
    obstacleN = np.array([55.0]) * scaleFactorN  # ft, left-bottom
    obstacleChi = np.array([0.0])  # rad
    obstacleLength = np.array([20]) * scaleFactorN  # ft
    obstacleWidth = np.array([25.1]) * scaleFactorE

    obstacleE = startPoint[0] - obstacleWidth / 2 # ft, left-bottom
    safetyMargin = 5.0 * scaleFactorE
    obstacleSafetyE = obstacleE - safetyMargin

    pathWidth = 2.0 * safetyMargin

# ---------------------------------------------------------------------
# debug

obstacleClass = obstacleInfo(obstaclePresent, obstacleE, obstacleN, obstacleChi,
                             obstacleWidth, obstacleLength, safetyMargin)
obstacle = obstacleClass()

print(nE,nN,nU)

if debug:
    if nE > 32:  # 16 - corrected on 10/11/2018
        sf_E = float(nE) / 32
        nE = int(nE / sf_E)
        obstacle.E = np.array([int(obstacle.E / sf_E)])
        obstacle.w = np.array([int(obstacle.w / sf_E)])
        startPoint[0] = startPoint[0] / sf_E
        endPoint[0] = endPoint[0] / sf_E
    else:
        sf_E = 1.0

    if nN > 128:
        sf_N = float(nN) / 128
        nN = int(nN / sf_N)
        obstacle.N = np.array([int(obstacle.N / sf_N)])
        obstacle.l = np.array([int(obstacle.l / sf_N)])
        startPoint[1] = startPoint[1] / sf_N
        endPoint[1] = endPoint[1] / sf_N
    else:
        sf_N = 1.0

    if nU > 8:
        sf_U = float(nU) / 8
        nU = int(nU / sf_U)
        nU_low = int(nU / sf_U)
        startPoint[2] = startPoint[2] / sf_U
        endPoint[2] = endPoint[2] / sf_U
    else:
        sf_U = 1.0

    print(nE, nN, nU)

# ---------------------------------------------------------------------

startPoint_ = np.append(startPoint, height)
endPoint_ = np.append(endPoint, height)

obstacleData = createObstacleData(nE, nN, nU, gridSize, obstacle)

# ---------------------------------------------------------------------
# Run LP

slow_convergence_test = 1

print('Laplacian planner calculation started ...')

tStart = time.time()
path, not_converged = laplacian( startPoint_, endPoint_, nE, nN, nU, nU_low,
                                 obstacleData, slow_convergence_test )
tElapsed = (time.time() - tStart)

print("Time taken = %*.1f sec" %(5, tElapsed))
# ---------------------------------------------------------------------

# Plot

# figure 1
plt.figure(1, figsize=(5, 7), dpi=100)

plt.ylabel('N [ft]')
plt.xlabel('E [ft]')
# plt.axis('equal')

for k in range(0,widthSpace):
    x = [k, k]
    y = [0, lengthSpace]
    plt.plot(x, y, color='lightgrey')


for k in range(0,lengthSpace,4):
    x = [0, widthSpace]
    y = [k, k]
    plt.plot(x, y, color='lightgrey')

# Detailed Path
plt.plot(path[0,:], path[1,:], linestyle='--', color='c')

plt.plot(startPoint[0], startPoint[1], marker='o', markersize=8, color='r')
plt.plot(endPoint[0], endPoint[1], marker='o', markersize=8, color='g')


if obstaclePresent == True:
    k = 0
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

plt.xlim([0, widthSpace])
plt.ylim([0, lengthSpace])

# ------------------------------------
# Results:

# 32 x 128:
# small = 21.1 sec (50)
# medium = 21.4 sec(50)
# large = 29.4 sec (75)

# 64 x 128:
# small = 64.9 sec (75)
# medium = 21.4 sec(50)
# large = 29.4 sec (75)

if False:

    sf = 21.1

    print("32 x 128")
    print(21.1/21.1)
    print(20.6/21.1)
    print(27.4/21.1)

    print("64 x 128")
    print(64.9/21.1)
    print(61.7/21.1)
    print(56.9/21.1)

    print("32 x 256")
    print(80.8/21.1)
    print(79.4/21.1)
    print(73.3/21.1)

    print("64 x 256")
    print(168.5/21.1)
    print(165.6/21.1)
    print(151.2/21.1)


plt.show()
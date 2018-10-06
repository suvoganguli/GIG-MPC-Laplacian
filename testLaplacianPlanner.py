import numpy as np
from laplacianPlanner import *
from obstacleData import *
import time
import matplotlib.pyplot as plt
import matplotlib.figure as fig
import matplotlib.patches as patches

# -----------------------------------------------------------------
# Create Grid

scaleFactorE = 1
scaleFactorN = 1
scaleFactorh = 1

widthSpace = 16 # ft
lengthSpace = 128  # ft
heightSpace = 8 # ft

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
startPoint = np.array([8 * scaleFactorE, 1 * scaleFactorN])  # E (ft), N (ft)
endPoint = np.array([8 * scaleFactorE, 127 * scaleFactorN])  # E (ft), N (ft)

startPoint_ = np.append(startPoint, height)
endPoint_ = np.append(endPoint, height)

# ---------------------------------------------------------------------
# Obstacle Data

obstaclePresent = True
safetyMargin = 5

obstacleN = np.array([54.0]) * scaleFactorN  # ft, left-bottom
obstacleChi = np.array([0.0])  # rad
obstacleLength = np.array([20]) * scaleFactorN  # ft
obstacleWidth = np.array([5]) * scaleFactorE + 0.1
obstacleE = startPoint[0] - obstacleWidth / 2 # ft, left-bottom

obstacleClass = obstacleInfo(obstaclePresent, obstacleE, obstacleN, obstacleChi,
                             obstacleWidth, obstacleLength, safetyMargin)
obstacle = obstacleClass()
obstacleData = createObstacleData(nE, nN, nU, gridSize, obstacle)

# ---------------------------------------------------------------------
# Run LP

slow_convergence_test = 1

tStart = time.time()
path, not_converged = laplacian( startPoint_, endPoint_, nE, nN, nU, nU_low, obstacleData, slow_convergence_test )
tElapsed = (time.time() - tStart)
print("Time taken = %*.1f sec" %(5, tElapsed))
# ---------------------------------------------------------------------

# Plot

# figure 1
plt.figure(1, figsize=(5, 7), dpi=100)

plt.ylabel('N [ft]')
plt.xlabel('E [ft]')
# plt.axis('equal')

# Detailed Path
plt.plot(path[0,:], path[1,:], linestyle='--', color='c')

plt.plot(startPoint[0], startPoint[1], marker='o', markersize=8, color='r')
plt.plot(endPoint[0], endPoint[1], marker='o', markersize=8, color='g')

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

plt.show()
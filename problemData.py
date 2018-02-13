import numpy as np
from utils import *

# Units
mph2fps = 4.4/3

# ----------------------------------------------------------
# USER INPUTS
# ----------------------------------------------------------

# Grid selection

scaleFactor = 1
widthSpace = 16 * scaleFactor  # ft
lengthSpace = 128 * scaleFactor  # ft
heightSpace = 8  # ft

gridSize = 1 # ft/unit
gridClass = createGrid(gridSize, lengthSpace, widthSpace, heightSpace)
grid = gridClass()

# Start and End Points
startPoint = np.array([7, 1]) * scaleFactor  # E (ft), N (ft)
endPoint = np.array([7, 115]) * scaleFactor  # E (ft), N (ft)


# expt:
# 'N' - number of MPC time steps
# 'T - time step
# 'ns' - number of states
# 'no' - number of obstacles

# default
N = 4
T = 0.4
ns = 4
no = 1

if no == 0:
    if N == 4:
        mpciterations = 36 #34
    elif N == 6:
        mpciterations = 34 #34
    elif N == 8:
        mpciterations = 32 # 32


elif no == 1:
    if N == 4:
        mpciterations = 40 #34
    elif N == 6:
        mpciterations = 34 #34
    elif N == 8:
        mpciterations = 32 # 32


elif no == 2:
    if N == 4:
        mpciterations = 50 # 50
    elif N == 6:
        mpciterations = 50 #34
    elif N == 8:
        mpciterations = 36 # 32



# Number of states
# ns = 2:
#   x0 = E, x1 = N
#   u0 = V, u1 = chi
# ns = 4:
#   x0 = E, x1 = N, x2 = V, x3 = chi
#   u0 = Vdot, u1 = chidot
# ns = 6:
#   x0 = E, x1 = N, x2 = V, x3 = chi, x5 = Vdot, x6 = chidot
#   u0 = Vddot, u1 = chiddot


# Obstacle Present (for MPC re-routing)
# (True or False)
obstaclePresent = False

# Detection Window
detectionWindow = {'L': 50, 'W': 11}

# Positon Index w.r.t. Path Sections
posIdx0 = {'number': 0}

# ----------------------------------------------------------

if ns == 2:

    # Ipopt settings
    nlpMaxIter = 100
    #mpciterations = 5

    # Kinematic Constraints
    E0 = startPoint[0]  # ft (North, long)
    N0 = startPoint[1]  # ft (East, lat)
    Chi0 = 0 * np.pi / 180  # rad
    V0 = 10 * mph2fps
    x0 = [E0, N0]  # E, N, V, Chi, Vdot, Chidot

    lb_VdotVal = -2  # fps3
    ub_VdotVal = 2  # fps3
    lb_ChidotVal = -20 * np.pi / 180  # rad/s2
    ub_ChidotVal = 20 * np.pi / 180  # rad/s2
    lataccel_maxVal = 0.25 * 32.2  # fps2
    useLatAccelCons = 0
    lb_V = 0.8 * V0
    ub_V = 1.2 * V0

    # Tracking Tuning and Data
    W_P = 1.0
    W_V = 1.0
    W_Vdot = 10.0
    W_Chidot = 1.0

    V_cmd = V0  # fps

    # Terminal constraint
    delta_yRoad = 0.5 * 5  # ft
    delta_yRoadRelaxed = 5  # ft, in safe zone
    delta_V = 1 * mph2fps  # fps

    # Path parameters
    pathWidth = 5.0 # ft


elif ns == 4:

    # Ipopt settings
    nlpMaxIter = 100


    # Kinematic Constraints
    E0 = startPoint[0]  # ft (North, long)
    N0 = startPoint[1]  # ft (East, lat)
    Chi0 = 0 * np.pi / 180  # rad
    V0 = 5 * mph2fps
    x0 = [E0, N0, V0, Chi0]  # E, N, V, Chi, Vdot, Chidot

    lb_VdotVal = -2  # fps3
    ub_VdotVal = 2  # fps3
    lb_ChidotVal = -20 * np.pi / 180  # rad/s2
    ub_ChidotVal = 20 * np.pi / 180  # rad/s2
    lataccel_maxVal = 0.25 * 32.2  # fps2
    useLatAccelCons = 1
    lb_V = 0.8 * V0
    ub_V = 1.2 * V0

    # Tracking Tuning and Data
    W_P = 1.0
    W_V = 1.0
    W_Vdot = 10.0
    W_Chidot = 1.0

    V_cmd = V0  # fps

    # Terminal constraint
    delta_yRoad = 0.5  # ft
    delta_yRoadRelaxed = 5  # ft, in safe zone
    delta_V = 1 * mph2fps  # fps

    # Path parameters
    pathWidth = 5.0 # ft

elif ns == 6:

    # Ipopt settings
    nlpMaxIter = 100
    #mpciterations = 5

    # Kinematic Constraints

    E0 = startPoint[0]  # ft (North, long)
    N0 = startPoint[1]  # ft (East, lat)
    Chi0 = 0*np.pi/180 # rad (w.r.t. North)
    V0 = 10*mph2fps
    x0 = [E0, N0, V0, Chi0, 0, 0]  # E, N, V, Chi, Vdot, Chidot
    lb_VddotVal = -2 # fps3
    ub_VddotVal = 2 # fps3
    lb_ChiddotVal = -30*np.pi/180 # rad/s2
    ub_ChiddotVal = 30*np.pi/180 # rad/s2
    lataccel_maxVal = 0.25*32.2 # fps2
    useLatAccelCons = 1
    lb_V = 0.8*V0
    ub_V = 1.2*V0

    # Tracking Tuning and Data
    W_P = 1.0      #1.0
    W_V = 1.0
    W_Vddot = 10.0   # 20.0
    W_Chiddot = 1.0 #0.1

    V_cmd = V0  # fps

    # Terminal constraint
    delta_yRoad = 0.5 # ft
    delta_yRoadRelaxed = 5 # ft, in safe zone
    delta_V = 1*mph2fps # fps

    # Path parameters
    pathWidth = 5.0 # ft

# ------------------------------------------------------------

# Obstacle Data

if no == 0:
    runOnce = False
    obstacleE = np.array([]) * scaleFactor # ft, left-bottom
    obstacleN = np.array([]) * scaleFactor # ft, left-bottom
    obstacleChi = np.array([]) * scaleFactor  # rad
    obstacleLength = np.array([]) * scaleFactor # ft
    obstacleWidth = np.array([]) * scaleFactor # ft

elif no == 1:
    runOnce = True
    obstacleE = np.array([3.0]) * scaleFactor # ft, left-bottom
    obstacleN = np.array([63.0]) * scaleFactor # ft, left-bottom
    obstacleChi = np.array([0.0]) * scaleFactor  # rad
    obstacleLength = np.array([4.0]) * scaleFactor # ft
    obstacleWidth = np.array([8.0]) * scaleFactor # ft

elif no == 2:
    runOnce = True
    obstacleE = np.array([4.0, 7.0]) * scaleFactor # ft, left-bottom
    obstacleN = np.array([31.0, 63.0]) * scaleFactor # ft, left-bottom
    obstacleChi = np.array([0.0, 0.0]) * scaleFactor  # rad
    obstacleLength = np.array([4.0, 4.0]) * scaleFactor # ft
    obstacleWidth = np.array([6.0, 6.0]) * scaleFactor # ft


# ------------------------------------------------------------

if ns == 2:
    # problem size
    nx = 2
    nu = 2
    ncons = 2 * N + 2  # running + lataccel + terminal constraint-y
    t0 = 0
    u0 = np.zeros([N, nu])
    # mpciterations = int(18*N/(6))

    # nlpData
    nlpPrintLevel = 0

    # State and Control indices
    idx_E = 0
    idx_N = 1

    idx_V = 1
    idx_Chi = 1

elif ns == 4:
    # problem size
    nx = 4
    nu = 2
    ncons = 2 * N + 2  # running + lataccel + terminal constraint-y
    t0 = 0
    u0 = np.zeros([N, nu])
    # mpciterations = int(18*N/(6))

    # nlpData
    nlpPrintLevel = 0

    # State and Control indices
    idx_E = 0
    idx_N = 1
    idx_V = 2
    idx_Chi = 3

    idx_Vdot = 0
    idx_Chidot = 1

elif ns == 6:
    # problem size
    nx = 6
    nu = 2
    ncons = 2*N + 4 #running + lataccel + V0 + terminal constraint-y + terminal constraint-V
    t0 = 0
    u0 = np.zeros([N,nu])
    #mpciterations = int(18*N/(6))

    # nlpData
    nlpPrintLevel = 0

    # State and Control indices
    idx_E = 0
    idx_N = 1
    idx_V = 2
    idx_Chi = 3
    idx_Vdot = 4
    idx_Chidot = 5

    idx_Vddot = 0
    idx_Chiddot = 1

else:
    print("Error in ns")

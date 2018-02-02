# Matlab (original): Mike Elgersma
# Python: Suvo Ganguli
# Feb 02, 2018

import numpy as np
import matplotlib.pyplot as plt

def laplacian( start_point, end_point, nxs, nys, nzs, nzs_low, obstacles, slow_convergence_test ):

    # ----------------  Initialization ----------------

    v0 = 0 # v0=0 and v_end=-1 gives dynamic range of -(10^(-308)) to -1
    v_end = -1
    gradient_sign = np.sign(v_end - v0)

    n_vec_exponents = [4, 3, 4]
    iter_max = 50

    n_vec = 2 ** np.array(n_vec_exponents)
    ny_factor = float(nxs) / float(nys) # POWER OF 2     (y grid_size) = (x grid size)/ny_factor
    nz_factor = float(nxs) / float(nzs) # POWER OF 2     (z grid_size) = (x grid size)/nz_factor

    #dt = float(nxs) / 128 # number of pixels per time step
    dt = 4
    tol = float(dt)

    # we want to make nt large enough for the path to run across the entire domain
    i_n = max(n_vec_exponents)
    nx_ = 2 ** i_n
    ny_ = int(nx_ / ny_factor)
    nz_ = int(nx_ / nz_factor)
    nt = int(np.linalg.norm(np.array([nx_, ny_, nz_]) / dt))
    nz_low = int(nz_/2)

    # ----------------  Build obstacles ----------------


    for i_n in np.arange(min(n_vec_exponents), max(n_vec_exponents)+1):
        nx = 2 ** i_n
        ny = int(nx / ny_factor) # this needs to be updated
        nz = int(nx / nz_factor)
        nz_low = int(nz * (nzs_low/ nzs))

        # nx = int(nx)
        # ny = int(ny)
        # nz = int(nz)
        # nz_low = int(nz_low)

        # Assemble obstacles
        obstacle = np.zeros([nx, ny, nz]) # obstacle array has logicals, (-1,0,1) for (goal, interior, obstacle)

        # If any point in fine obstacles block (that maps to point in coarse
        # obstacle array) contains a one, then corresponding point in coarse
        # obstacle array is set to one.
        # for i in np.arange(1, nx+1):
        #     ii = np.arange( 1 + (i-1)*nxs/nx, i*nxs/nx + 1)
        #
        #     for j in np.arange(1,ny+1):
        #         jj =   np.arange( 1+(j-1)*nys/ny, j*nys/ny + 1)
        #
        #         for k in np.arange(1,nz+1):
        #             kk = np.arange( 1+(k-1)*nzs/nz, k*nzs/nz + 1)
        #
        #             if sum ( sum ( obstacles[ii-1][jj-1][kk-1] ) ) > 0:
        #                 obstacle[i-1][j-1][k-1] =  1

        for i in range(nx):
            i1 = i * nxs / nx
            i2 = (i + 1) * nxs / nx

            for j in range(ny):
                j1 = j * nys / ny
                j2 = (j + 1) * nys / ny

                for k in range(nz):
                    k1 = k * nzs / nz
                    k2 = (k + 1) * nzs / nz

                    if sum(sum(sum(obstacles[ i1:i2, j1:j2 , k1:k2 ]))) > 0:
                        obstacle[i][j][k] = 1


        # Add outer domain bounday to obstacle
        for i in range(nx):
            for j in range(ny):
                obstacle[i,j,0] = 1
                if (j < ny / 2):
                    obstacle[i, j, nz_low-1] = 1
                else:
                    obstacle[i, j, nz-1] = 1

        for i in range(nx):
            for k in range(nz):
                obstacle[i,0,k] = 1
                obstacle[i,ny-1,k] = 1

        for k in range(nz):
            for j in range(ny):
                obstacle[0,j,k] = 1
                obstacle[nx-1,j,k] = 1

        scl = 2 ** (max(n_vec_exponents)-i_n)
        end_point_scl = end_point / scl
        obstacle[int(end_point_scl[0]),int(end_point_scl[1]),int(end_point_scl[2])] = -1 # identify end point

        if (nx == 4):
            obstacle_n4 = obstacle
        elif (nx == 8):
            obstacle_n8 = obstacle
        elif(nx == 16):
            obstacle_n16 = obstacle
        elif(nx == 32):
            obstacle_n32 = obstacle
        elif(nx == 64):
            obstacle_n64 = obstacle
        elif(nx == 128):
            obstacle_n128 = obstacle

    # ---------------- Build potential field ---------------

    # Initialize solution on first grid
    n_old = n_vec[0]
    v_old = v0 * np.ones([int(n_old), int(n_old / ny_factor), int(n_old / nz_factor)])

    for i_n in np.arange(1, len(n_vec)):

        nx = n_vec[i_n]
        ny = int(nx / ny_factor)
        nz = int(nx / nz_factor)
        nz_low = int(nz * (nzs_low / nzs))


        if(nx==4):
            obstacle = obstacle_n4
        elif(nx==8):
            obstacle = obstacle_n8
        elif(nx==16):
            obstacle = obstacle_n16
        elif(nx==32):
            obstacle = obstacle_n32
        elif(nx==64):
            obstacle = obstacle_n64
        elif(nx==128):
            obstacle = obstacle_n128

        # Initialize new solution with interpolated old solution of previous grid size
        v = v0 * np.ones(obstacle.shape) # Gives correct v on obstacles, v_end overwritten later
        if (nx > n_old):
            for i in np.arange(1,int(nx)):
                for j in np.arange(1,int(ny)):
                    for k in np.arange(1,int(nz)):
                        if (obstacle[i,j,k] == 0): # interior points
                            i_ = min(nx - 1, max(1, int(np.floor(i / 2))))      # CHK1
                            j_ = min(ny - 1, max(1, int(np.floor(j / 2))))
                            k_ = min(nz - 1, max(1, int(np.floor(k / 2))))
                            v[i,j,k] = v_old[i_,j_,k_]  # update to intepolate later
        elif(nx < n_old):
            for i in np.arange(1, nx - 1):
                for j in np.arange(1, ny - 1):
                    for k in np.arange(1, nz - 1):
                        if (obstacle[i,j,k] == 0):  # interior points
                            i_ = min(nx - 1, max(1, int(np.floor(i * 2))))      # CHK2
                            j_ = min(ny - 1, max(1, int(np.floor(j * 2))))
                            k_ = min(nz - 1, max(1, int(np.floor(k * 2))))
                            v[i,j,k] = v_old[i_,j_,k_]  # update to average later

        else:
            v = v_old

        # Compute solution, v(i,j,k)
        # Iterate to get Laplacian solution for potential

        None
        for iter in range(iter_max):
            for i in np.arange(1, nx-1):
                for j in np.arange(1, ny-1):
                    for k in np.arange(1, nz-1):
                        if (obstacle[i,j,k] == 1):
                            v[i,j,k] = v0
                        elif (obstacle[i,j,k] == -1):
                            v[i,j,k] = v_end
                        else:
                            v[i,j,k] = ( v[i-1,j,k] + v[i+1,j,k] +
                                           v[i,j-1,k] + v[i,j+1,k] +
                                           v[i,j,k-1] + v[i,j,k+1] ) / 6

        v_old = v
        n_old = nx

    # ---------------- Convergence Test ---------------

    not_converged = False
    if (slow_convergence_test == 1):
        ijk_local_min = np.array([np.NaN, np.NaN, np.NaN])
        ijk_local_max = np.array([np.NaN, np.NaN, np.NaN])
        for i in np.arange(1,nx-1):
            for j in np.arange(1,ny-1):
                for k in np.arange(1,nz-1):
                    if (obstacle[i,j,k] == 0):
                        v_min = min( v[i - 1,j,k], v[i + 1,j,k],
                                     v[i,j - 1,k], v[i,j + 1,k],
                                     v[i,j,k - 1], v[i,j,k + 1] )
                        v_max = max( v[i - 1,j,k], v[i + 1,j,k],
                                     v[i,j - 1,k], v[i,j + 1,k],
                                     v[i,j,k - 1], v[i,j,k + 1] )

                        if v[i,j,k] <= v_min:
                            not_converged = True
                            ijk_local_min = [i, j, k]
                        elif v[i,j,k] >= v_max:
                            not_converged = True
                            ijk_local_max = [i, j, k]

        if not_converged == True:
            print('Not Converged')
        else:
            print('Converged')

    # ----------------  Find Laplacian path ---------------

    # Compute path along gradient of potential


    path = np.NaN * np.ones([nt, 3])
    path[0] = start_point

    for it in np.arange(1, nt):
        i = max(2, min(nx - 1, np.int(path[it - 1, 0])))
        j = max(2, min(ny - 1, np.int(path[it - 1, 1])))
        k = max(2, min(nz - 1, np.int(path[it - 1, 2])))

        dv_dx = (
                    sum( sum( v[i + 1,j - 1:j + 1,k - 1:k + 1] ) ) / 9
                  - sum( sum( v[i - 1,j - 1:j + 1,k - 1:k + 1] ) ) / 9
                ) /2
        dv_dy = (
                    sum( sum( v[i - 1:i + 1,j + 1,k - 1:k + 1] ) ) / 9
                  - sum( sum( v[i - 1:i + 1,j - 1,k - 1:k + 1] ) ) / 9
                ) /2
        dv_dz = (
                    sum( sum( v[i - 1:i + 1,j - 1:j + 1,k + 1] ) ) / 9
                  - sum( sum( v[i - 1:i + 1,j - 1:j + 1,k - 1] ) ) / 9
                ) /2
        gradient_vec = np.array([dv_dx, dv_dy, dv_dz])
        gradient_vec = gradient_vec[:,None]
        path[it] = path[it - 1] + dt * gradient_sign * np.squeeze(gradient_vec) / (1e-306 + np.linalg.norm(gradient_vec))

        if np.linalg.norm(path[it]-end_point) < tol:
            for k in np.arange(it+1,nt):
                path[k] = end_point


    return path, not_converged, nx, ny, nz, nz_low, v

# ----------------------------------------------------------------
# Testing

nxs = 16
nys = 32
nzs = 8
obstacles = np.zeros([nxs, nys, nzs])
w = 4
l = 4
x0 = 5
y0 = 14
for j in range(w):
    for k in range(l):
        obstacles[x0+j,y0+k,:] = 1

obstacles[:,:,0] = 1

start_point = np.array([7, 1.1, 1],dtype='float')  # [8, 2.1, 2]
end_point = np.array([7, 30, 1],dtype='float')  # [8, 31, 2]

nzs_low = nzs
slow_convergence_test = 1

scale = 512 / nxs # (feet/grid_point)   eg scale=4 ft/grid_point when nxs=128

path, not_converged, nx, ny, nz, nz_low, v = \
    laplacian( start_point, end_point, nxs, nys, nzs, nzs_low, obstacles, slow_convergence_test )

East = path[:,0]
North = path[:,1]

plt.plot(East, North, marker='x', markersize=4, color='b')
plt.plot(East, North, marker='x', markersize=4, color='b')
plt.plot(start_point[0], start_point[1], marker='o', markersize=4, color='r')
plt.plot(end_point[0], end_point[1], marker='o', markersize=4, color='g')
plt.grid('True')
plt.ylabel('N [ft]')
plt.xlabel('E [ft]')
plt.axis('equal')
plt.grid('True')
plt.show()
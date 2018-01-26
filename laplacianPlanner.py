# Matlab (original): Mike Elgersma
# Python: Suvo Ganguli

import numpy as np

def laplacian( start_point, end_point, nxs, nys, nzs, nzs_low, obstacles, slow_convergence_test ):

    # ----------------  Initialization ----------------

    v0 = 0 # v0=0 and v_end=-1 gives dynamic range of -(10^(-308)) to -1
    v_end = -1
    gradient_sign = np.sign(v_end - v0)

    if (nxs == 128):
        if (nzs == 32):
            n_vec_exponents = [6, 5, 4, 5, 4, 5, 6, 7]
            iter_max = 25 # nx = 2 ^ 4 = 16 coarse grid.runtime =?s for nzs=32,
        elif (nzs == 16):
            n_vec_exponents = [6, 5, 4, 5, 4, 5, 6, 7]
            iter_max = 25 # nx = 2 ^ 4 = 16 coarse grid.runtime = 6s for nzs=16, CONVERGED
        elif(nzs == 8): # Artificial ceiling at half height, to force OAV below rooftops
            n_vec_exponents = [6, 5, 6, 5, 6, 7]
            iter_max = 50
    elif(nxs == 64):
        if (nzs == 16):
            n_vec_exponents = [5, 4, 5, 4, 5, 6, 6]
            iter_max = 25
        elif(nzs == 8):
            n_vec_exponents = [5, 4, 5, 4, 5, 6, 6]
            iter_max = 25
        elif(nzs == 4):
            n_vec_exponents = [5, 4, 5, 4, 5, 6, 6]
            iter_max = 50

    n_vec = 2.0 ** np.array(n_vec_exponents)
    ny_factor = nxs / nys # POWER OF 2     (y grid_size) = (x grid size)/ny_factor
    nz_factor = nxs / nzs # POWER OF 2     (z grid_size) = (x grid size)/nz_factor

    dt = float(nxs) / 128 # number of pixels per time step

    # we want to make nt large enough for the path to run across the entire domain
    i_n = max(n_vec_exponents)
    nx_ = 2 ** i_n
    ny_ = nx_ / ny_factor
    nz_ = nx_ / nz_factor
    nt = round(np.linalg.norm(np.array([nx_, ny_, nz_]) / dt))



    # ----------------  Build obstacles ----------------


    for i_n in np.arange(min(n_vec_exponents), max(n_vec_exponents)):
        nx = 2 ** i_n
        ny = nx / ny_factor # this needs to be updated
        nz = nx / nz_factor
        nz_low = nz * (nzs_low/ nzs)

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

                    print(i,j,k)
                    if sum(sum(sum(obstacles[ i1:i2 ][ j1:j2 ][ k1:k2 ]))) > 0:
                        obstacle[i][j][k] = 1


        # Add outer domain bounday to obstacle
        for i in range(nx):
            for j in range(ny):
                obstacle[i][j][1] = 1
                if (j < ny / 2):
                    obstacle[i-1][j-1][nz_low-1] = 1
                else:
                    obstacle[i-1][j-1][nz-1] = 1

        for i in range(nx):
            for k in range(nz):
                obstacle[i][0][k] = 1
                obstacle[i][ny-1][k] = 1

        for k in range(nz):
            for j in range(ny):
                obstacle[0][j][k] = 1
                obstacle[nx-1][j][k] = 1

            obstacle[end_point[0]][end_point[1]][end_point[2]] = -1; # identify end point

        if (nx == 8):
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
    v_old = v0 * np.ones([n_old, n_old / ny_factor, n_old / nz_factor])

    for i_n in np.arange(1, len(n_vec)):
        nx = n_vec[i_n]
        ny = nx / ny_factor
        nz = nx / nz_factor
        nz_low = nz * (nzs_low / nzs)

        if(nx==8):
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
        for i in np.arange(1,nx-1):
            for j in np.arange(1,ny-1):
                for k in np.arange(1,nz-1):
                    if (obstacle[i][j][k] == 0): # interior points
                        i_ = min(nx - 1, max(2, np.floor(i / 2)))
                        j_ = min(ny - 1, max(2, np.floor(j / 2)))
                        k_ = min(nz - 1, max(2, np.floor(k / 2)))
                        v[i][j][k] = v_old[i_][j_][k_]  # update to intepolate later
    elif(nx < n_old):
        for i in np.arange(1, nx - 1):
            for j in np.arange(1, ny - 1):
                for k in np.arange(1, nz - 1):
                    if (obstacle[i][j][k] == 0):  # interior points
                        i_ = min(nx - 1, max(2, np.floor(i * 2)));
                        j_ = min(ny - 1, max(2, np.floor(j * 2)));
                        k_ = min(nz - 1, max(2, np.floor(k * 2)));
                        v[i][j][k] = v_old[i_][j_][k_]  # update to average later

    else:
        v = v_old

    # Compute solution, v(i,j,k)
    # Iterate to get Laplacian solution for potential

    for iter in range(iter_max):
        for i in np.arange(1, nx - 1):
            for j in np.arange(1, ny - 1):
                for k in np.arange(1, nz - 1):
                    if (obstacle[i][j][k] == 1):
                        v[i][j][k] = v0
                    elif (obstacle[i][j][k] == -1):
                        v[i][j][k] = v_end
                    else:
                        v[i][k][k] = ( v[i-1][j][k] + v[i+1][j][k] +
                                       v[i][j-1][k] + v[i][j+1][k] +
                                       v[i][j][k-1] + v[i][j][k+1] ) / 6

    v_old = v
    n_old = nx

    # ---------------- Convergence Test ---------------

    not_converged = 0
    if (slow_convergence_test == 1):
        ijk_local_min = np.array([np.NaN, np.NaN, np.NaN])
        ijk_local_max = np.array([np.NaN, np.NaN, np.NaN])
        for i in np.arange(1,nx-1):
            for j in np.arange(1,ny-1):
                for k in np.arange(1,nz-1):
                    if (obstacle[i][j][k] == 0):
                        v_min = min( v[i - 1][j][k] + v[i + 1][j][k] +
                                     v[i][j - 1][k] + v[i][j + 1][k] +
                                     v[i][j][k - 1] + v[i][j][k + 1] )
                        v_max = max( v[i - 1][j][k] + v[i + 1][j][k] +
                                     v[i][j - 1][k] + v[i][j + 1][k] +
                                     v[i][j][k - 1] + v[i][j][k + 1] )

                        if v[i][j][k] <= v_min:
                            not_converged = 1
                            ijk_local_min = [i, j, k]
                        elif v[i][j][k] >= v_max:
                            not_converged = 1
                            ijk_local_max = [i, j, k]

    # ----------------  Find Laplacian path ---------------

    # Compute path along gradient of potential


    path = np.NaN * np.ones([nt, 3])
    path[1] = start_point

    for it in np.arange(1, nt+1):
        i = max(2, min(nx - 1, np.floor(path(1, it - 1))))
        j = max(2, min(ny - 1, np.floor(path(2, it - 1))))
        k = max(2, min(nz - 1, np.floor(path(3, it - 1))))

        dv_dx = (
                    sum( sum( v[i + 1][j - 1:j + 1][k - 1:k + 1] ) ) / 9
                  - sum( sum( v[i - 1][j - 1:j + 1][k - 1:k + 1] ) ) / 9
                ) /2
        dv_dy = (
                    sum( sum( v[i - 1:i + 1][j + 1][k - 1:k + 1] ) ) / 9
                  - sum( sum( v[i - 1:i + 1][j - 1][k - 1:k + 1] ) ) / 9
                ) /2
        dv_dz = (
                    sum( sum( v[i - 1:i + 1][j - 1:j + 1][k + 1] ) ) / 9
                  - sum( sum( v[i - 1:i + 1][j - 1:j + 1][k - 1] ) ) / 9
                ) /2
        gradient_vec = np.array([dv_dx, dv_dy, dv_dz])
        gradient_vec = gradient_vec[:,None]
        path[it] = path[it - 1] + dt * gradient_sign * gradient_vec / (1e-306 + np.linalg.norm(gradient_vec))

    return path, not_converged, nx, ny, nz, nz_low, v

# ----------------------------------------------------------------
# Testing

start_point = np.array([30, 10, 2],dtype='float')
end_point = np.array([60, 10, 2],dtype='float')

obstacles = np.zeros([64, 64, 16])

nxs = 64
nys = nxs
nzs = int(8*(float(nxs)/128))
nzs_low = nzs
slow_convergence_test = 0

scale = 512 / nxs # (feet/grid_point)   eg scale=4 ft/grid_point when nxs=128

path, not_converged, nx, ny, nz, nz_low, v = \
    laplacian( start_point, end_point, nxs, nys, nzs, nzs_low, obstacles, slow_convergence_test )


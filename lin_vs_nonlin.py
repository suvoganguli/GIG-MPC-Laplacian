import numpy as np

def nonlin(uk, xk, T):

    xkp1 = [0, 0, 0, 0]
    xkp1[0] = xk[0] + T * xk[2] * np.sin(xk[3]) # Edot
    xkp1[1] = xk[1] + T * xk[2] * np.cos(xk[3]) # Ndot
    xkp1[2] = xk[2] + T * uk[0]                 # Vdot
    xkp1[3] = xk[3] + T * uk[1]                 # Chidot

    return xkp1


def lin(u0_k, x0_k, del_u_k, del_x_k, T):

    E0_k   = x0_k[0]
    N0_k   = x0_k[1]
    V0_k   = x0_k[2]
    Chi0_k = x0_k[3]

    u10_k = u0_k[0]
    u20_k = u0_k[1]

    del_E_k   = del_x_k[0]
    del_N_k   = del_x_k[1]
    del_V_k   = del_x_k[2]
    del_Chi_k = del_x_k[3]

    del_u1_k = del_u_k[0]
    del_u2_k = del_u_k[1]

    del_E_kp1   = del_E_k + T * (np.sin(Chi0_k) * del_V_k + V0_k * np.cos(Chi0_k) * del_Chi_k)  # Edot
    del_N_kp1   = del_N_k + T * (np.cos(Chi0_k) * del_V_k - V0_k * np.sin(Chi0_k) * del_Chi_k)  # Edot
    del_V_kp1   = del_V_k + T * del_u1_k
    del_Chi_kp1 = del_Chi_k + T * del_u2_k


    x_del_kp1 = np.array([del_E_kp1, del_N_kp1 ])

    return x_del_kp1


V0 = 10
chi0 = 30 * np.pi / 180

xk = [0, 0, V0, chi0]
xkp1 = [0, 0, 0, 0]






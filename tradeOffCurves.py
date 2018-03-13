import numpy as np
import matplotlib.pyplot as plt


# row-names = N = 4, 6, 8
# col-names = ns = 0, 1, 2
CT_data_ns4 = np.array([[0.24,	0.79,	0.70],
                    [0.68,	6.54,	6.04],
                    [1.40,	16.11,	18.76],
                    [2.77, 44.26, 48.42]])/0.24



CT_data_ns6 = np.array([[0.23,	0.60,	0.54],
                    [0.63,	3.34,	3.81],
                    [1.36,	15.93,	17.07]])/0.24


# -------------------------------------------------------------
# N vs CT (ns4)

model = 3

if model == 1:
    a = 2.0
    b = 0.5
    N0 = 3.8

    n_N = 3
    n_no = 3
    CT_model_ns4 = np.zeros((n_N, n_no))
    error_ns4 = np.zeros((n_N, n_no))

    N = np.array([4, 6, 8])
    no = np.array([0, 1, 2])

    for i in range(n_no):
        for j in range(n_N):

            CT_model_ns4[i,j] = a * (1 - np.exp(-b * (N[j] - N0))) * (no[i] + 1)**2
            error_ns4[i,j] = CT_data_ns4[i,j] - CT_model_ns4[i,j]

elif model == 2:
    a = 0.8
    b = 0.5
    N0 = 4

    n_N = 4
    n_no = 3
    CT_model_ns4 = np.zeros((n_N, n_no))
    error_ns4 = np.zeros((n_N, n_no))

    N = np.array([4, 6, 8, 10])
    no = np.array([0, 1, 2])

    for i in range(n_N):
        for j in range(n_no):
            CT_model_ns4[i, j] = a * (2**(N[i] - N0)/2) * (no[j] + 1)
            error_ns4[i, j] = CT_data_ns4[i, j] - CT_model_ns4[i, j]

elif model == 3:
    a = 0.4/3/0.24
    b = 1.2
    c = 1.5
    d = 1
    N0 = 4

    n_N = 4
    n_no = 3
    CT_model_ns4 = np.zeros((n_N, n_no))
    error_ns4 = np.zeros((n_N, n_no))

    N = np.array([4, 6, 8, 10])
    no = np.array([0, 1, 2])

    for i in range(n_N):
        for j in range(n_no):
            CT_model_ns4[i, j] = a * b**( c * (2*N[i]+3) ) * (1 - np.exp(-d*no[j]))
            error_ns4[i, j] = CT_data_ns4[i, j] - CT_model_ns4[i, j]

print('ns4 data:')
print(CT_data_ns4)
print('')
print(CT_model_ns4)
print('')
print(error_ns4)
print('')
print(np.linalg.norm(error_ns4))
print('')


ix_no = 1
ix_N = 2
plt.figure(1, figsize=(5, 7))
plt.subplot(211)
plt.plot(N,CT_model_ns4[:,ix_no],N,CT_data_ns4[:,ix_no])
plt.xlabel('N')
plt.ylabel('CPU Time (normalized)')
plt.grid(True)
plt.subplot(212)
plt.plot(no,CT_model_ns4[ix_N,:],no,CT_data_ns4[ix_N,:])
plt.xlabel('no')
plt.ylabel('CPU Time (normalized)')
plt.grid(True)

# -------------------------------------------------------------
# N vs CT (ns4)

a = 0.4/3/0.24
b = 1.2
c = 1.55
d = 1
N0 = 4

n_N = 3
n_no = 3
CT_model_ns6 = np.zeros((n_N, n_no))
error_ns6 = np.zeros((n_N, n_no))

N = np.array([4, 6, 8])
no = np.array([0, 1, 2])

for i in range(n_N):
    for j in range(n_no):
        CT_model_ns6[i, j] = a * b**( c * (2*N[i]+3) ) * (1 - np.exp(-d*no[j])) - 8
        error_ns6[i, j] = CT_data_ns6[i, j] - CT_model_ns6[i, j]

print('ns6 data:')
print(CT_data_ns6)
print('')
print(CT_model_ns6)
print('')
print(error_ns6)
print('')
print(np.linalg.norm(error_ns6))
print('')


ix_no = 1
ix_N = 2
plt.figure(2, figsize=(5, 7))
plt.subplot(211)
plt.plot(N,CT_model_ns6[:,ix_no],N,CT_data_ns6[:,ix_no])
plt.xlabel('N')
plt.ylabel('CPU Time (normalized)')
plt.grid(True)
plt.subplot(212)
plt.plot(no,CT_model_ns4[ix_N,:],no,CT_data_ns4[ix_N,:])
plt.xlabel('no')
plt.ylabel('CPU Time (normalized)')
plt.grid(True)


# -------------------------------------------------------------
# Smoothness (ns=4, no=2)

data = np.array([0.25, 0.89, 1.44, 2.04])  # ft
N = np.array([4,6,8,10])

plt.figure(3)
plt.plot(N,data,color='b')
plt.plot(N,data,marker='o',markersize=4,color='b')
plt.xlabel('N')
plt.ylabel('Cornering Distance [ft]')
plt.grid(True)


# -------------------------------------------------------------
# N vs intg(Vdot.dt), intg(Chidot.dt), intg(latAccel.dt)

data_Vdot_ns4 = np.array([14.3, 12.03, 8.92, 6.33, ])  # ft
data_Chidot_ns4 = np.array([178.76, 140.94, 101.41, 71.05])  # deg
data_latAcel_ns4 = np.array([0.56, 0.46, 0.34, 0.25])  # fps

# note for ns=6 and N=4, the trajectory is unstable and terminated early
data_Vdot_ns6 = np.array([4.63, 8.42, 7.23 ])
data_Chidot_ns6 = np.array([126.05, 173.03, 140.37 ])
data_latAcel_ns6 = np.array([0.42, 0.58, 0.5])  # fps

N_ns4 = np.array([4,6,8,10])
N_ns6 = np.array([6,8,10])

plt.figure(4, figsize=(5, 7))
plt.subplot(311)
plt.plot(N,data_Vdot_ns4,color='b')
plt.plot(N,data_Vdot_ns4,marker='o',markersize=4,color='b')
plt.ylabel('Intg(Vdot.dt) [ft]')
plt.grid(True)

plt.subplot(312)
plt.plot(N,data_Chidot_ns4,color='b')
plt.plot(N,data_Chidot_ns4,marker='o',markersize=4,color='b')
plt.ylabel('Intg(Chidot.dt) [deg]')
plt.grid(True)

plt.subplot(313)
plt.plot(N,data_latAcel_ns4,color='b')
plt.plot(N,data_latAcel_ns4,marker='o',markersize=4,color='b')
plt.ylabel('Intg(latAccel.dt) [fps]')
plt.grid(True)
plt.xlabel('N')

# plt.figure(5, figsize=(5, 7))
# plt.subplot(211)
# plt.plot(N,data_Vdot,color='b')
# plt.plot(N,data_Vdot,marker='o',markersize=4,color='b')
# plt.ylabel('Intg(Vdot.dt) [ft]')
# plt.grid(True)
#
# plt.subplot(212)
# plt.plot(N,data_Chidot,color='b')
# plt.plot(N,data_Chidot,marker='o',markersize=4,color='b')
# plt.ylabel('Intg(Chidot.dt) [deg]')
# plt.grid(True)


plt.show()
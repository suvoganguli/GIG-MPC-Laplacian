import numpy as np
import matplotlib.pyplot as plt


# row-names = N = 4, 6, 8
# col-names = ns = 0, 1, 2
CT_data_ns4 = np.array([[0.24,	0.79,	0.70],
                    [0.68,	6.54,	6.04],
                    [1.40,	16.11,	18.76]])



CT_data_ns6 = np.array([[0.23,	0.60,	0.54],
                    [0.63,	3.34,	3.81],
                    [1.36,	15.93,	17.07]])


a = 2.0
b = 0.5
N0 = 3.8

n_N = 3
n_no = 3
CT_model_ns6 = np.zeros((n_N, n_no))
error_ns6 = np.zeros((n_N, n_no))

N = np.array([4, 6, 8])
no = np.array([0, 1, 2])

for i in range(n_N):
    for j in range(n_no):

        CT_model_ns6[i,j] = a * (1 - np.exp(-b * (N[j] - N0))) * (no[i] + 1)**2
        error_ns6[i,j] = CT_data_ns6[i,j] - CT_model_ns6[i,j]

#print(CT_data_ns6)
#print(CT_model_ns6)
print(error_ns6)
print(np.linalg.norm(error_ns6))
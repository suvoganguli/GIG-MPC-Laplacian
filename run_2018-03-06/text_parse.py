import numpy as np
import matplotlib.pyplot as plt

inputfiles = ['output_N6_Tp4_ns4_no1.txt',
              'output_N6_Tp4_ns6_no1.txt']

n_iter = 38-1
data_iter = np.array([np.zeros(n_iter), np.zeros(n_iter)])

for i in range(len(inputfiles)):
    inputfile = open(inputfiles[i])

    text = inputfile.readlines()

    nlines = len(text)
    num_array = np.array([], dtype = int)

    for k in range(nlines):
        line = text[k]
        word = line[0:5]

        try:
            num = int(word)
            num_array = np.concatenate([num_array,[num]])
        except ValueError:
            None

    m = len(num_array)
    iter_array = np.array([], dtype = int)

    for k in range(m-1):
        diff = num_array[k+1] - num_array[k]
        if diff < 0:
            iter_array = np.concatenate([iter_array, [num_array[k]]])

    data_iter[i] = iter_array

    print(inputfiles[i])
    print(len(iter_array))
    print(iter_array)
    print(sum(iter_array))
    print('\n')

plt.figure
plt.plot(range(n_iter),data_iter[0])
plt.plot(range(n_iter),data_iter[1])
plt.grid(True)
plt.ylabel('Solver Iterations')
plt.xlabel('Trajectory Steps')
plt.legend(['ns=4','ns=6'])
plt.grid(True)
plt.show()
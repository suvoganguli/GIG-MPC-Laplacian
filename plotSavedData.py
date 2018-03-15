import os
import glob
import printPlots
import matplotlib.pyplot as plt
import numpy as np
from utils import *

def createPlots(mode, pathObj = None, dirNames = None, fileNames=None):
    # mode = 0 : get input from user
    # mode = 1 : use hardcoded files

    oldpwd = os.getcwd()
    if mode == 0:

        listDir = glob.glob("run*")
        print(listDir)
        dirNames = raw_input("Input directory name: ")

        os.chdir(dirNames)

        listFile = glob.glob("*.txt")
        print(listFile)
        fileName = raw_input("Input file name (*.txt): ")

        listFile = glob.glob("*.pkl")
        print(listFile)
        filePkl = raw_input("Input file name (*.pkl): ")

        cpuMeanTime = printPlots.plotSavedData(fileName, filePkl, delim=" ", header=False)
        print('CPU time:')
        print(cpuMeanTime)

        os.chdir(oldpwd)


    elif mode == 1:

        oldpwd = os.getcwd()

        n = len(fileNames)
        cpuMeanTime = np.zeros(n)
        noVec = np.zeros(n)
        NVec = np.zeros(n)
        TVec = np.zeros(n)

        for k in range(n):
            print(k)
            os.chdir(dirNames[k])

            fileName = fileNames[k]
            cpuMeanTime[k] = printPlots.plotSavedData(fileName, pathObj, delim=" ", header=False)
            noVec[k] = np.array(fileName[22]).astype(np.int)
            NVec[k] = np.array(fileName[9:11]).astype(np.int)
            TVec[k] = np.array(fileName[14]).astype(np.float)/10

            os.chdir(oldpwd)

        min_cpuMeanTime = 0.248
        print('CPU time:')
        print(cpuMeanTime)
        print('CPU time - normalized:')
        print(cpuMeanTime / min_cpuMeanTime)

        plt.figure(10)
        plt.plot(noVec, cpuMeanTime, marker='x')
        plt.xlabel('Number of Obstacles (no)')
        plt.ylabel('Average CPU time [sec]')
        plt.grid(True)

        plt.figure(11)
        plt.plot(NVec, cpuMeanTime, marker='x')
        plt.xlabel('Number of Time Steps (N)')
        plt.ylabel('Average CPU time [sec]')
        plt.grid(True)

        plt.figure(12)
        plt.plot(TVec, cpuMeanTime, marker='x')
        plt.xlabel('Time Step (T)')
        plt.ylabel('Average CPU time [sec]')
        plt.grid(True)

        plt.show()


    return None

# -----------------------------------------------------

mode = 1    # user setting

if mode == 0:
    createPlots(mode)

elif mode == 1:

    # case:
    # 1 - run_2018-03-06 (comprehensive runs for different N, ns and no)
    # 2 - run_2018-03-14 (tradeoff charts for various T)
    # 3 - run_2018-03-14 vs run_2018-03-15 (V = 5 mph vs 10 mph)
    case = 3

    if case == 1:

        dirNames = ['run_2018-03-06',
                    'run_2018-03-06',
                    'run_2018-03-06',
                    'run_2018-03-06']

        fileNames = ['logFile_N04_Tp4_ns4_no2.txt',
                     'logFile_N06_Tp4_ns4_no2.txt',
                     'logFile_N08_Tp4_ns4_no2.txt',
                     'logFile_N10_Tp4_ns4_no2.txt'
                     ]

        # dirNames = ['run_2018-03-14',
        #             'run_2018-03-14',
        #             'run_2018-03-14']

        # fileNames = ['logFile_N04_Tp4_ns6_no2.txt',
        #              'logFile_N06_Tp4_ns6_no2.txt',
        #              'logFile_N08_Tp4_ns6_no2.txt'
        #              ]

        # dirNames = ['run_2018-03-14',
        #             'run_2018-03-14']

        # fileNames = ['logFile_N06_Tp4_ns4_no2.txt',
        #              'logFile_N06_Tp4_ns6_no2.txt',
        #              ]

        fileSettings = dirNames[0] + '/' + 'settings_N04_Tp4_ns4_no2.txt' # used for V0 only (V0=Vcmd)
        f = file(fileSettings, 'r')
        cols, indexToName = getColumns(f, delim=" ", header=False)
        V_cmd = cols[12] # V_cmd is stored for information only

    if case == 2:

        dirNames = ['run_2018-03-14',
                    'run_2018-03-14',
                    'run_2018-03-14']

        fileNames = ['logFile_N06_Tp2_ns4_no2.txt',
                     'logFile_N06_Tp4_ns4_no2.txt',
                     'logFile_N06_Tp6_ns4_no2.txt']

        # dirNames = ['run_2018-03-14',
        #             'run_2018-03-14']

        # fileNames = ['logFile_N09_Tp4_ns4_no2.txt',
        #              'logFile_N04_Tp9_ns4_no2.txt']


        fileSettings = dirNames[0] + '/' + 'settings_N06_Tp6_ns4_no2.txt' # used for V0 only (V0=Vcmd)
        f = file(fileSettings, 'r')
        cols, indexToName = getColumns(f, delim=" ", header=False)
        V_cmd = cols[12] # V_cmd is stored for information only

    elif case == 3:

        dirNames = ['run_2018-03-14',   # V_cmd = 5 mph
                    'run_2018-03-15']   # V_cmd = 10 mph

        fileNames = ['logFile_N04_Tp4_ns4_no2.txt',
                     'logFile_N04_Tp4_ns4_no2.txt'
                     ]

        n = len(dirNames)
        fileSettings = []
        V_cmd = np.zeros(n)

        for k in range(n):
            file_tmp = dirNames[k] + '/' + 'settings_N04_Tp4_ns4_no2.txt' # used for varying V0 (V0=Vcmd)
            fileSettings.append(file_tmp)

            f = file(fileSettings[k], 'r')
            cols, indexToName = getColumns(f, delim=" ", header=False)
            V_cmd[k] = np.array(cols[12]).astype(np.float)

    filePkl = dirNames[0] + '/' + 'pathDict_no2_NoPopup.pkl'  # used for path as a function of no
    pathObj = loadpkl(filePkl)
    pathObj['V_cmd'] = V_cmd  # V_cmd is stored for information only

    createPlots(mode, pathObj, dirNames, fileNames)

    dummy = raw_input('Press Enter to Continue: ')

# -----------------------------------------------------
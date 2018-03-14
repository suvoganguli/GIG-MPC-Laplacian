import os
import glob
import printPlots
import matplotlib.pyplot as plt
import numpy as np
from utils import *

def createPlots(mode, pathObj = None, dirName = None, fileNames=None):
    # mode = 0 : get input from user
    # mode = 1 : use hardcoded files

    oldpwd = os.getcwd()
    if mode == 0:

        listDir = glob.glob("run*")
        print(listDir)
        dirName = raw_input("Input directory name: ")

        os.chdir(dirName)

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

        n = len(fileNames)
        cpuMeanTime = np.zeros(n)
        noVec = np.zeros(n)
        NVec = np.zeros(n)

        oldpwd = os.getcwd()
        os.chdir(dirName)

        n = len(fileNames)
        for k in range(n):
            print(k)
            fileName = fileNames[k]
            cpuMeanTime[k] = printPlots.plotSavedData(fileName, pathObj, delim=" ", header=False)
            noVec[k] = np.array(fileName[22]).astype(np.int)
            NVec[k] = np.array(fileName[9:11]).astype(np.int)

        os.chdir(oldpwd)

        T = np.array(fileName[14]).astype(np.float)/10
        V_cmd = pathObj['V_cmd']  # strictly speaking the horz dist [ft] will vary with V, but always has a constant N

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
        plt.plot(NVec*T*V_cmd, cpuMeanTime, marker='x')
        plt.xlabel('Horizon Distance [ft]')
        plt.ylabel('Average CPU time [sec]')
        plt.grid(True)

        plt.show()


    return None

# -----------------------------------------------------

mode = 1    # user setting

if mode == 0:
    createPlots(mode)

elif mode == 1:

    dirName = 'run_2018-03-14'

    # fileNames = ['logFile_N04_Tp4_ns4_no2.txt',
    #              'logFile_N06_Tp4_ns4_no2.txt',
    #              'logFile_N08_Tp4_ns4_no2.txt',
    #              'logFile_N10_Tp4_ns4_no2.txt'
    #              ]

    # fileNames = ['logFile_N04_Tp4_ns6_no2.txt',
    #              'logFile_N06_Tp4_ns6_no2.txt',
    #              'logFile_N08_Tp4_ns6_no2.txt'
    #              ]

    # fileNames = ['logFile_N06_Tp4_ns4_no2.txt',
    #              'logFile_N06_Tp4_ns6_no2.txt',
    #              ]

    # fileNames = ['logFile_N04_Tp8_ns4_no2.txt'
    #              ]

    fileNames = ['logFile_N06_Tp2_ns4_no2.txt',
                 'logFile_N06_Tp4_ns4_no2.txt',
                 'logFile_N06_Tp6_ns4_no2.txt']


    filePkl = dirName + '/' +'pathDict_no2_NoPopup.pkl'
    fileSettings = dirName + '/' + 'settings_N06_Tp6_ns4_no2.txt' # used for V0 only (V0=Vcmd)

    f = file(fileSettings, 'r')
    cols, indexToName = getColumns(f, delim=" ", header=False)
    V_cmd = np.array(cols[12]).astype(np.float)

    pathObj = loadpkl(filePkl)
    pathObj['V_cmd'] = V_cmd

    createPlots(mode, pathObj, dirName, fileNames)

    dummy = raw_input('Press Enter to Continue: ')

# -----------------------------------------------------
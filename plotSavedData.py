import os
import glob
import printPlots
import matplotlib.pyplot as plt
import numpy as np
from utils import *

def createPlots(mode, pathObj=None, dirName = None, fileNames=None):
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

        oldpwd = os.getcwd()
        os.chdir(dirName)

        n = len(fileNames)
        for k in range(n):
            print(k)
            fileName = fileNames[k]
            cpuMeanTime[k] = printPlots.plotSavedData(fileName, pathObj, delim=" ", header=False)
            noVec[k] = np.array(fileName[22]).astype(np.int)

        os.chdir(oldpwd)

        min_cpuMeanTime = 0.248
        print('CPU time:')
        print(cpuMeanTime)
        print('CPU time - normalized:')
        print(cpuMeanTime / min_cpuMeanTime)

        plt.figure(10)
        plt.plot(noVec, cpuMeanTime, marker='x')
        plt.xlabel('Number of Obstacles')
        plt.ylabel('Average CPU time [sec]')
        plt.grid(True)
        plt.show()




    return None

# -----------------------------------------------------

mode = 1    # user setting

if mode == 0:
    createPlots(mode)

elif mode == 1:

    plots = 'N'

    #if plots == 'N'

    dirName = 'run_2018-03-06'
    fileNames = ['logFile_N04_Tp4_ns4_no2.txt',
                 'logFile_N06_Tp4_ns4_no2.txt',
                 'logFile_N08_Tp4_ns4_no2.txt',
                 'logFile_N10_Tp4_ns4_no2.txt']

    filePkl = dirName + '/' +'pathDict_no2_NoPopup.pkl'

    pathObj = loadpkl(filePkl)

    createPlots(mode, pathObj, dirName, fileNames)

    dummy = raw_input('Press Enter to Continue: ')

# -----------------------------------------------------
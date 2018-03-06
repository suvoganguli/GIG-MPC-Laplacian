import os
import datetime
import printPlots
import glob

def createPlots(mode, dirName = None, fileNames=None):
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
        fileName = raw_input("Input file name (*.txt):")

        ns = fileName[16]

        cols, indexToName = printPlots.plotSavedData(fileName, ns, delim=" ", header=False)

        os.chdir(oldpwd)

        return cols, indexToName


    elif mode == 1:

        oldpwd = os.getcwd()

        os.chdir(dirName)

        ns = int(fileNames[0][17])

        for fileName in fileNames:
            cols, indexToName = printPlots.plotSavedData(fileName, ns, delim=" ", header=False)

        os.chdir(oldpwd)

        return cols, indexToName

# -----------------------------------------------------

mode = 1    # user setting

if mode == 0:
    createPlots(mode)

elif mode == 1:
    dirName = 'run_2018-02-28'
    fileNames = ['logFile_N4_Tp4_ns4_no2.txt',
                 'logFile_N4_Tp4_ns6_no2.txt']

    createPlots(mode, dirName, fileNames)

# -----------------------------------------------------
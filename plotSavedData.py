import os
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

        cols, indexToName = printPlots.plotSavedData(fileName, delim=" ", header=False)

        os.chdir(oldpwd)

        return cols, indexToName


    elif mode == 1:

        printPlots.test()

        oldpwd = os.getcwd()
        os.chdir(dirName)

        n = len(fileNames)
        for k in range(n):
            print(k)
            fileName = fileNames[k]
            cols, indexToName = printPlots.plotSavedData(fileName, delim=" ", header=False)

        os.chdir(oldpwd)

        return cols, indexToName

# -----------------------------------------------------

mode = 1    # user setting

if mode == 0:
    createPlots(mode)

elif mode == 1:
    dirName = 'run_2018-03-06'
    fileNames = ['logFile_N6_Tp4_ns4_no0.txt',
                 'logFile_N6_Tp4_ns4_no1.txt',
                 'logFile_N6_Tp4_ns4_no2.txt']

    createPlots(mode, dirName, fileNames)
    dummy = raw_input('Press Enter to Continue: ')

# -----------------------------------------------------
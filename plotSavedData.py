import os
import datetime
import printPlots
import glob

oldpwd = os.getcwd()

# listDir = glob.glob("run*")
# print(listDir)
# dirName = raw_input("Input directory name: ")
dirName = 'run_2018-03-01'

os.chdir(dirName)

# listFile = glob.glob("*.txt")
# print(listFile)
# fileName = raw_input("Input file name (*.txt):")
fileName = 'logFile_N4_Tp4_ns6_no2.txt'

cols, indexToName = printPlots.plotSavedData(fileName, delim=" ", header=False)

print(cols)

os.chdir(oldpwd)

None



import csv
import math

import rclpy
from numpy.linalg import norm
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt



# filePath = "/home/omax/Downloads/straight path parking try 1 (traj 11) 2022-06-2714.47.43.csv"
filePath = "/home/omax/Downloads/straight path 1 parking try 2 (traj 11) 2022-06-2714.49.55.csv"
file = open(filePath)
type(file)
csvreader = csv.reader(file)
# reads ths headers
header = []
header = next(csvreader)
print(header)

xAcc = np.array([[0, 0]])
yAcc = np.array([[0, 0]])
resultant = np.array([[0, 0]])
for row in csvreader:
    npPointX = np.array([float(row[0]), float(row[1])])
    npPointy = np.array([float(row[0]), float(row[2])])
    npPointResultant = np.array([float(row[0]), math.sqrt(float(row[1])**2 + float(row[2])**2)])
    xAcc = np.append(xAcc, [npPointX], axis=0)
    yAcc = np.append(yAcc, [npPointy], axis=0)
    resultant = np.append(resultant, [npPointResultant], axis=0)
xAcc = np.delete(xAcc, [[0, 0]], axis=0)
yAcc = np.delete(yAcc, [[0, 0]], axis=0)
resultant = np.delete(resultant, [[0, 0]], axis=0)

y = resultant[:, 1]
window = 200
average_y = np.array([0])
for ind in range(len(y) - window + 1):
    mean = np.array([np.mean(y[ind:ind+window])])
    average_y = np.append(average_y, mean, axis=0)

for ind in range(window - 1):
    # average_y.insert(0, np.nan)
    average_y = np.append(np.array([np.nan]), average_y,  axis=0)

# plt.figure()
# plt.plot(resultant[:, 0], average_y)

# trajectoryLoaded = True
print(xAcc[0])
plt.figure()
plt.plot(xAcc[:, 0], xAcc[:, 1])
plt.figure()
plt.plot(yAcc[:, 0], yAcc[:, 1])
plt.figure()
# plt.plot(resultant[:, 0], resultant[:, 1])
plt.plot(resultant[:, 0][yAcc[:, 1]>=0], resultant[:, 1][yAcc[:, 1]>=0],'.')
plt.xlabel("Time (s)")
plt.ylabel("Resultant Acceleration (g)")
plt.title("Resultant acceleration given positive forward acceleration")
plt.grid()
# plt.savefig(fname = "Resultan Acceleration Figure run 2" ,dpi = 1000)
# plt.figure()
plt.show()
file.close()

# def main():
#
#
#
# if __name__ == '__main__':
#     main()
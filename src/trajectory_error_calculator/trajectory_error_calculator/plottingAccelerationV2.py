import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

accelerationFile = open('/home/omax/Downloads/straight path parking try 1 (traj 11) 2022-06-2714.47.43.csv')
accelerationValues = pd.read_csv(accelerationFile)

time = accelerationValues['time']
xAcc = accelerationValues['gFx']
yAcc = accelerationValues['gFy']
resultant = np.sqrt(xAcc ** 2 + yAcc ** 2)

# y = resultant.copy
# window = 200
# average_y = np.array([0])
# for ind in range(len(y) - window + 1):
#     mean = np.array([np.mean(y[ind:ind+window])])
#     average_y = np.append(average_y, mean, axis=0)
#
# for ind in range(window - 1):
#     # average_y.insert(0, np.nan)
#     average_y = np.append(np.array([np.nan]), average_y,  axis=0)
#
# plt.figure()
# plt.plot(resultant[:, 0][yAcc[:, 1]>=0], average_y[yAcc[:, 1]>=0])

print(xAcc[0])
plt.figure()
plt.plot(time, xAcc)
plt.grid
plt.xlabel('Time (s)')
plt.ylabel('X Acceleration (g)')

plt.figure()
plt.plot(time, yAcc)
plt.grid
plt.xlabel('Time (s)')
plt.ylabel('Y Acceleration (g)')

plt.figure()
plt.plot(time, resultant)
plt.xlabel("Time (s)")
plt.ylabel("Resultant Acceleration (g)")
# plt.title("Resultant acceleration given positive forward acceleration")
plt.grid()

plt.figure()
plt.plot(time[yAcc >= 0], resultant[yAcc >= 0], '.')
plt.xlabel("Time (s)")
plt.ylabel("Resultant Acceleration (g)")
plt.title("Resultant acceleration given positive forward acceleration")
plt.grid()
# plt.savefig(fname = "Resultant Acceleration Figure run 2" ,dpi = 1000)
# plt.figure()
plt.show()

above1 = resultant[resultant > 0.1]
forwardAcc = resultant[yAcc >= 0]
forwardAbove1 = forwardAcc[forwardAcc>0.1]
print(str(len(forwardAcc)) + " " + str(len(resultant)) + " " + str(len(above1)) + " " + str(len(forwardAbove1)))
percentageAbove1 = len(above1)/len(resultant) * 100
percentageForwardAbove1 = len(forwardAbove1) / len(forwardAcc) * 100

print("Percentage over limit = " + str(percentageAbove1))

print("Percentage forward over limit = " + str(percentageForwardAbove1))

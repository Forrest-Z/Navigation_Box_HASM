import math


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


filePathError = '/home/omax/AIIM/navigation_box/Pics/Trajectory Error Calc/Complex Trajectory with many curves/lat panda on min lookahead 3 ratio 2/Error.csv'
pathError = open(filePathError,'r+')
pathRealPose = open('/home/omax/AIIM/navigation_box/Pics/Trajectory Error Calc/Complex Trajectory with many curves/lat panda on min lookahead 3 ratio 2/RealPose.csv')
pathRecordedPose = open('/home/omax/AIIM/navigation_box/src/mqtt_pubsub/data/13_ParkingComplexPath.csv')



recordedPose = pd.read_csv(pathRecordedPose)

# recordedPose = np.loadtxt(pathRecordedPose, delimiter=",", dtype='float')
error = np.loadtxt(pathError, delimiter=",",dtype='float')
error *= 100
realPose = np.loadtxt(pathRealPose, delimiter=",",dtype='float')

# error = error[len(error) - len(realPose): len(error)]
# print(len(error))
# print(len(realPose))
# pathError.close()
# pathError = open(filePathError, "w")
# pathError.truncate()
# pathError.close()
# pathError = open(filePathError, "a")
# #
# with pathError as csvfile:
# #     # np.savetxt(csvfile, self.error, delimiter=',', header='Error', fmt='%s', comments='')
#     np.savetxt(csvfile, error, delimiter=',',
#                fmt='%s', comments='')

x = realPose[:,0]
y = realPose[:,1]
w = realPose[:,2]
z = realPose[:,3]

siny = 2 * (w*z + x*y)
cosy = 1 - 2 * (y*y + z*z)
yaw = np.arctan2(siny,cosy)

# siny_cosp = 2 * (w * z + x * y);
#     cosy_cosp = 1 - 2 * (y * y + z * z);
#     yaw = atan2(siny_cosp, cosy_cosp);

plt.figure("Yaw")
plt.plot(yaw)
plt.show()

plt.figure('1')
# plt.plot(error,'.',markersize=1)
plt.hist(error, bins=np.arange(min(error), max(error), (max(error) - min(error))/20), density=True)
plt.legend('Error Histogram')
plt.xlabel('cm')
plt.ylabel('Ratio')
# plt.savefig(fname = "Error Histogram Complex Trajectory" ,dpi = 1000)
plt.show()


window_size = 20
i = 0
# Initialize an empty list to store moving averages
moving_averages = []
# Loop through the array t o
# consider every window of size 3
while i < len(error) - window_size + 1:
    # Calculate the average of current window
    window_average = np.sum(error[i:i + window_size]) / window_size
    # Store the average of current
    # window in moving average list
    moving_averages.append(window_average)
    # Shift window to right by one position
    i += 1

plt.figure('Moving Average of Error')
plt.plot(moving_averages, linewidth=1)
plt.grid(which='both')
plt.title('Moving Average, window size = ' + str(window_size))
plt.xlabel('Index')
plt.ylabel('Trajectory Error (cm)')
# plt.savefig(fname = "Moving Average of Error Complex Trajectory" ,dpi = 1000)
plt.show()


plt.figure('2')
plt.title('Trajectories comparison')
plt.plot(recordedPose.iloc[:, 2], recordedPose.iloc[:, 3], '.',  markersize= 3)
plt.plot(realPose[:, 4], realPose[:, 5], '.', markersize=1)
plt.legend(['Prerecorded', 'Autonomously Driven'])
plt.xlabel('X coordinate (m)')
plt.ylabel('Y coordinate (m)')
# plt.savefig(fname = "Trajectories Comparison Complex Trajectory" ,dpi = 1000)
plt.show()


# plt.figure('2')
# plt.title('Trajectories comparison')
# # plt.plot(recordedPose.iloc[:, 2], recordedPose.iloc[:, 3], '.',  markersize= 3)
# plt.plot3(realPose[:, 4], realPose[:, 5], moving_averages, '.', markersize=1)
# plt.legend(['Prerecorded', 'Autonomously Driven'])
# plt.xlabel('X coordinate (m)')
# plt.ylabel('Y coordinate (m)')
# plt.show()


# This code is developed by Alireza & Mahmoud

import csv
import math
import os
from traceback import print_tb

import rclpy
from numpy.linalg import norm
from rclpy.node import Node
import numpy as np

# Importing the required ros2 message types
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion


import matplotlib.pyplot as plt

from aiim_autoware_msgs.msg import VehicleKinematicState
#include <aiim_autoware_msgs/msg/vehicle_kinematic_state.hpp>


class trajectoryErrorCalculator(Node):
    """
        A Node that compares the planned trajectory vs the actually driven trajectory and calculates the deviation error
    """

    def __init__(self):
        super().__init__('trajectory_error_calculator')

        # Ros2 Publishers, Subscribers

        self.trajectoryLoaded = False
        self.recordedTrajectoryPoints = np.array([[0, 0]])
        self.error = np.array([0])
        self.subscription = self.create_subscription(PoseStamped, '/ndt_pose', self.onPoseUpdated, 10)
        self.subscription
        self.vehicleStateSub = self.create_subscription(VehicleKinematicState, '/vehicle_state', self.onStateUpdated, 10)
        self.trajectoryFileNameSub = self.create_subscription(String, '/trajectory_file_path', self.onTrajectoryFilePath, 10)
        self.trajectoryFileNameSub

        # with open('Error.csv', 'a') as csvfile:
        #     np.savetxt(csvfile, delimiter=',', header='Error', fmt='%s', comments='')


        self.realTrajectoryPointsList = []
        # self.readCSVFile(os.path.expanduser("~/Software/BuurAutonoom/navigation_box/src/mqtt_pubsub/data/1_ShortC2.csv"))
        

        f = open("Error.csv", "w")
        f.truncate()
        f.close()

        f = open("RealPose.csv", "w")
        f.truncate()
        f.close()

    def onStateUpdated(self, msg: VehicleKinematicState):
        # print(msg.delta.translation.x)
        # print(msg.state.acceleration_mps2)
        print(msg.delta._rotation)
        print(msg.delta.translation)
        # print(msg)
        
    def onPoseUpdated(self, msg: PoseStamped):
        """
            Call back for ros2 subscriber (AIIM) to retrieve location information from NDT node
        """
        if self.trajectoryLoaded:
            point = np.array([msg.pose.position.x, msg.pose.position.y])
            self.calculateError(point)
            self.realTrajectoryPointsList.append(msg.pose.position)
            with open('RealPose.csv', 'a') as csvfile:
                # np.savetxt(csvfile, self.error, delimiter=',', header='Error', fmt='%s', comments='')
                np.savetxt(csvfile, np.array([[msg.pose.orientation.x, msg.pose.orientation.y , msg.pose.orientation.w, msg.pose.orientation.z, msg.pose.position.x, msg.pose.position.y]]), delimiter=',', fmt='%s', comments='')



    def onTrajectoryFilePath (self, msg:String):
        # self.trajectoryFileName = msg
        # self.readCSVFile(os.path.expanduser("~/AIIM/navigation_box/src/mqtt_pubsub/data/1_ShortC2.csv"))
        self.readCSVFile(msg.data)

    def readCSVFile(self, filePath: String):
        try:
            file = open(filePath)
            type(file)
            csvreader = csv.reader(file)
            # reads ths headers
            header = []
            header = next(csvreader)
            # if header[2] != "x" or header[3] != "y":
            #     print("csv file is not in the right format")
            #     return


            self.recordedTrajectoryPoints = np.array([[0, 0]])
            for row in csvreader:
                npPoint = np.array([float(row[2]), float(row[3])])
                self.recordedTrajectoryPoints = np.append(self.recordedTrajectoryPoints, [npPoint], axis=0)
            self.recordedTrajectoryPoints = np.delete(self.recordedTrajectoryPoints, [[0, 0]], axis=0)
            self.trajectoryLoaded = True
            file.close()
            print("File:", filePath, " loaded")
        except:
            print("Cannot open the trajectory file: ", filePath, "\n", "Please include the full path of the csv file")


    def calculateError(self, carPositionTrajectorypoint: np.array):
        minDistance1 = 10000
        minDistance2 = 10000

        point1 = np.array([0, 0])
        point2 = np.array([0, 0])
        for trajectoryPoint in self.recordedTrajectoryPoints:
            # dist = math.sqrt((trajectoryPoint[0] - point[0]) ** 2 + (trajectoryPoint[1] - point[1]) ** 2)
            dist = np.linalg.norm(trajectoryPoint - carPositionTrajectorypoint)
            if dist < minDistance1:
                minDistance2 = minDistance1
                minDistance1 = dist
                point2 = point1
                point1 = trajectoryPoint
            elif dist < minDistance2:
                minDistance2 = dist
                point2 = trajectoryPoint

        print("min Distance 1 = ", minDistance1)
        print("min Distance 2 = ", minDistance2)
        print("point 1 = ", point1)
        print("point 2 = ", point2)
        print("Car position = ", carPositionTrajectorypoint)
        d = np.abs(norm(np.cross(point2 - point1, point1 - carPositionTrajectorypoint))) / norm(point2 - point1)

        if self.error[0] == 0:
            self.error = [d]
        else:
            self.error = np.append(self.error, [d], axis=0)

        # print(self.error)
        print("Distance to nearest segment = ", d)
        if len(self.error) > 1:
            print("Maximum deviation = ", self.error.max(axis=0))
        rmse = np.linalg.norm(self.error) / np.sqrt(len(self.error))
        print("Root mean square error", rmse, "\n")

        with open('Error.csv', 'a') as csvfile:
            # np.savetxt(csvfile, self.error, delimiter=',', header='Error', fmt='%s', comments='')
            np.savetxt(csvfile, [d], delimiter=',', fmt='%s', comments='')


def main(args=None):
    rclpy.init(args=args)

    errorCalculator = trajectoryErrorCalculator()

    rclpy.spin(errorCalculator)
    # print("ending node operation")
    # plt.figure()
    # plt.plot(errorCalculator.error)
    # plt.show()
    # plt.savefig(fname = "Error Calculation" ,dpi = 1000)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    errorCalculator.client.loop_stop()
    errorCalculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

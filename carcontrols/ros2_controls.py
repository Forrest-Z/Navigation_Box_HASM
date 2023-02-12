#!/usr/bin/env python

# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import zmq
import threading
import time
from copy import copy
from selfdrive.services import service_list
from cereal import car
import selfdrive.messaging as messaging
from selfdrive.car.car_helpers import get_car
import datetime
import csv

# from csv_logger import CsvLogger


"""
This process reads messages published by autowared.py to the autoware zmq socket and ultimately controls the car via
Openpilot
"""


x = datetime.datetime.now()
UDPLogsFolder = "/home/carpc/Software/BuurAutonoom/UDPLogs"
# UDPLogDate = str(x.year) + str(x.month) + str(x.day)
UDPLogDate = str(x.date())
UDPLogTime = str(x.hour) + ":" + str(x.minute) + ":" + str(x.second)
UDPlogFileName = UDPLogsFolder + "/" + UDPLogDate + " " + UDPLogTime + " UDPLog.csv"
# filePathError = "/home/omax/AIIM/navigation_box/UDPLogs/" + str(x.year) + "_" + str(x.month) + "_" + str(x.day) + " " + str(x.hour) + ":" + str(x.minute) + ":" + str(x.second) + " UDPLog.csv"
# UDPLogPath = open(UDPlogFileName, "a")

header = ['Date', 'Autonomous Driving State', 'Final Steering Command', 'Final acceleration Command', 'Velocity', 'Yaw Rate', 'AD Steering Command', 'AD Acceleration Command']
# CSVheader = ",".join(header)

with open(UDPlogFileName, "w") as f:
    writer = csv.writer(f)
    writer.writerow(header)

# filename = 'logs/log.csv'
# delimiter = ','
# level = logging.INFO
# custom_additional_levels = ['logs_a', 'logs_b', 'logs_c']
# fmt = f'%(asctime)s{delimiter}%(levelname)s{delimiter}%(message)s'
# datefmt = '%Y/%m/%d %H:%M:%S'
# max_size = 1024  # 1 kilobyte
# max_files = 4  # 4 rotating files
# header = ['Date', 'Autonomous Driving State', 'Final Steering Command', 'Final acceleration Command', 'Velocity', 'Yaw Rate', 'AD Steering Command', 'AD Acceleration Command']
#
# csvlogger = CsvLogger(filename=UDPlogFileName,
#                       header=header)


def is_main_thread_active():
    """ Check if the main thread is still active. Return True if it is. False if it isn't"""
    # In Python 3.4+, you can use threading.main_thread().is_alive() instead
    return any((i.name == "MainThread") and i.is_alive()
               for i in threading.enumerate())


class UdpController:
    def __init__(self):
        self.init_openpilot_messaging()
        self.prevLogTime = time.time()
        self.logTime = 1.0
        self.autonomous_enable_status = False
        # Maximum velocity is set to 40 km/h. We have to convert this to m/s
        self.accel_cmd = 0.0
        self.steer_cmd = 0.0

        # Get car interface and parameters
        self.CI, self.CP = get_car(self.logcan_sub, self.sendcan_pub, None)

        self.CC = car.CarControl.new_message()

    def init_openpilot_messaging(self):
        self.context = zmq.Context()
        self.poller = zmq.Poller()

        # Receiving Openpilot messages
        self.logcan_sub = messaging.sub_sock(
            self.context,
            service_list['can'].port)
        self.command_sub = messaging.sub_sock(
            self.context,
            service_list['udpInMsg'].port,
            conflate=True,
            poller=self.poller)

        # Publishing Openpilot messages
        self.carstate_pub = messaging.pub_sock(
            self.context,
            service_list['carState'].port)
        self.carcontrol_pub = messaging.pub_sock(
            self.context,
            service_list['carControl'].port)
        self.sendcan_pub = messaging.pub_sock(
            self.context,
            service_list['sendcan'].port)
        self.as_state_pub = messaging.pub_sock(
            self.context,
            service_list['autonomousState'].port)

    def accelerate(self, acceleration):
        """
        Accelerate the car via Openpilot interfaces

        :param acceleration: acceleration sent to the car in m/s^2
        :param self.CC: Car Controller
        :param CI: Car Interface
        :return:
        """
        # We convert our acceleration command into two parts
        # Of which 1 will always be 0.
        accel_cmd = max(acceleration, 0.)
        brake_cmd = max(-acceleration, 0.)

        # This line can be used to limit acceleration for smoother driving
        # Default in openpilot is 0.2
        # For 5gmobix, we keep it the same as we have our own velocity controller already
        self.CC.actuators.gas = min(accel_cmd, 1.0)
        self.CC.actuators.brake = brake_cmd

    def check_as_enabled(self, CS):
        # Check for enable/disable signals
        # cruise_en becomes True when cruise control is activated by the driver (push on/off + lever down)
        # NOTE: cruise_en becomes True when cruise control is re-activated
        # after a fail (lever up)
        self.autonomous_enable_status = CS.cruiseState.enabled

        # Check for safety driver override
        # Currently, override is only given if the brake is pressed or the steer is actuated with sufficient torque.
        # Pressing the gas currently does not disable the autonomous system to
        # allow the driver to give a little more gas to the car during
        # demos (ACC like behaviour)
        if CS.steeringPressed or CS.brakePressed:  # or CS.gasPressed
            self.autonomous_enable_status = False

    def check_poller(self):
        self.new_data = False
        for op_socket, event in self.poller.poll(0):
            self.new_data = True
            if op_socket is self.command_sub:
                command_data = messaging.recv_one(op_socket)
                self.steer_cmd = command_data.udpInMsg.steer
                self.accel_cmd = command_data.udpInMsg.accel

    def command_car_interface(self):
        self.CC.hudControl.visualAlert = 0
        self.CC.hudControl.setSpeed = 15
        self.CC.cruiseControl.cancel = False
        self.CC.enabled = self.autonomous_enable_status
        self.CI.apply(self.CC)

    def broadcast_as_state(self):
        as_send = messaging.new_message()
        as_send.init('autonomousState')
        as_send.autonomousState.enabled = self.autonomous_enable_status
        self.as_state_pub.send(as_send.to_bytes())

    def broadcast_car_state(self, CS):
        cs_send = messaging.new_message()
        cs_send.init('carState')
        cs_send.carState = copy(CS)
        self.carstate_pub.send(cs_send.to_bytes())

    def broadcast_car_control(self):
        cc_send = messaging.new_message()
        cc_send.init('carControl')
        cc_send.carControl = copy(self.CC)
        self.carcontrol_pub.send(cc_send.to_bytes())

    def run(self):
        while is_main_thread_active():
            self.check_poller()

            CS = self.CI.update(self.CC)

            self.check_as_enabled(CS)
            self.accelerate(self.accel_cmd)

            # self.CC.actuators.steerAngle = self.steer_cmd / 0.8
            # self.CC.actuators.steer = self.CC.actuators.steerAngle / 520.


            # works with 7 km/h
            # self.CC.actuators.steerAngle = self.steer_cmd
            # self.CC.actuators.steer = self.CC.actuators.steerAngle / 360

            self.CC.actuators.steerAngle = self.steer_cmd
            self.CC.actuators.steer = self.CC.actuators.steerAngle / 180

            # self.CC.actuators.steerAngle = self.steer_cmd / 0.2
            # self.CC.actuators.steer = self.CC.actuators.steerAngle / 8020.

            # self.write_to_logs(CS)

            self.command_car_interface()

            # broadcast messages
            self.broadcast_as_state()
            self.broadcast_car_state(CS)
            self.broadcast_car_control()

            LogValues = [ x.now(),
                          self.autonomous_enable_status,
                          self.CC.actuators.steer,
                          self.CC.actuators.gas - self.CC.actuators.brake,
                          CS.vEgo,
                          CS.yawRate,
                          self.steer_cmd,
                          self.accel_cmd]
            #
            # csvlogger.log(msg= LogValues)

            # logMessage = ""

            # logMessage = ",".join([str(i) for i in LogValues])
            # print("Log Message = " + logMessage)
            print(self.prevLogTime)
            if (time.time() - self.prevLogTime) > self.logTime:
                with open(UDPlogFileName, "a") as f:
                    writer = csv.writer(f)
                    writer.writerow(LogValues)
                self.prevLogTime = time.time()


            print(
                "Autonomous Driving State",
                self.autonomous_enable_status,
                "Final Steering Command",
                self.CC.actuators.steer,
                "Final acceleration Command",
                self.CC.actuators.gas -
                self.CC.actuators.brake,
                "Current Velocity",
                CS.vEgo,
                "Current Yaw Rate",
                CS.yawRate,
                "AD Steering Command",
                self.steer_cmd,
                "AD Acceleration Command",
                self.accel_cmd)



            time.sleep(0.02)


if __name__ == "__main__":
    udp_controller = UdpController()
    udp_controller.run()
    # all_logs = csvlogger.get_logs(evaluate=False)
    # for log in all_logs:
    #     print(log)

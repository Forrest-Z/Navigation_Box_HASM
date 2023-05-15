#!/usr/bin/env python
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import zmq
import time
import math
import argparse
import threading
from copy import copy
from selfdrive.services import service_list
from cereal import car
import selfdrive.messaging as messaging
from selfdrive.car.car_helpers import get_car
from selfdrive.controls.lib.vehicle_model import VehicleModel


"""
This process reads messages published by autowared.py to the autoware zmq socket and ultimately controls the car via
Openpilot
"""


def is_main_thread_active():
    """ Check if the main thread is still active. Return True if it is. False if it isn't"""
    # In Python 3.4+, you can use threading.main_thread().is_alive() instead
    return any((i.name == "MainThread") and i.is_alive()
               for i in threading.enumerate())


class AutowareController:
    def __init__(self, use_ctrl_cmd=True):
        context = zmq.Context()
        self.poller = zmq.Poller()
        self.use_ctrl_cmd = use_ctrl_cmd

        logcan = messaging.sub_sock(context, service_list['can'].port)
        self.autoware_sock = messaging.sub_sock(
            context,
            service_list['autoware'].port,
            conflate=True,
            poller=self.poller)
        self.autoware_cmd_sock = messaging.sub_sock(
            context,
            service_list['autowareCommand'].port,
            conflate=True,
            poller=self.poller)
        self.geofencing_sock = messaging.sub_sock(
            context,
            service_list['autowareGeofencing'].port,
            conflate=True,
            poller=self.poller)
        self.obstacles_sock = messaging.sub_sock(
            context,
            service_list['autowareObstacles'].port,
            conflate=True,
            poller=self.poller)

        self.carstate = messaging.pub_sock(
            context, service_list['carState'].port)
        self.carcontrol = messaging.pub_sock(
            context, service_list['carControl'].port)
        sendcan = messaging.pub_sock(
            context, service_list['sendcan'].port)

        self.enabled = False  # autonomous enable status

        # Get car interface and parameters
        self.CI, self.CP = get_car(logcan, sendcan, None)

        self.VM = VehicleModel(self.CP)
        self.CC = car.CarControl.new_message()

        self.target_speed = 0.0
        self.target_yaw_rate = 0.0
        self.target_acceleration = 0.0
        self.target_steering_angle = 0.0

        self.geofencing_detected = False
        self.geofencing_speed = 1.5
        self.geofencing_timeout = 5.0

        self.obstacle_detected = False
        self.obstacle_distance = -1
        self.max_deceleration = -0.8  # m/s2
        self.obstacle_distance_buffer = 9

    def accelerate(self, acceleration):
        """
        Accelerate the car via Openpilot interfaces

        :param acceleration: acceleration sent to the car in m/s^2
        :param CC: Car Controller
        :param CI: Car Interface
        :return:
        """
        accel_cmd = max(acceleration, 0.)
        # limit acceleration for smoother driving to 0.2 instead of 1.0 limit in
        # openpilot
        self.CC.actuators.gas = min(accel_cmd, 0.2)
        # brake is limited at 1.0 by openpilot
        self.CC.actuators.brake = max(-acceleration, 0.)
        self.CI.apply(self.CC)

    def speed_controller(self):
        """
        P controller

        :param target_speed: target speed in m/s
        :param CC: Car Controller
        :param CI: Car Interface
        :return:
        """
        Kp = 0.4
        speed_error_threshold = 0.01

        # get current car state
        self.CS = self.CI.update(self.CC)
        current_speed = self.CS.vEgo

        speed_error = self.target_speed - current_speed
        if abs(speed_error) > speed_error_threshold:
            self.accelerate(Kp * speed_error)

    def check_geofencing(self):
        if self.geofencing_detected:
            self.target_speed = min(self.target_speed, self.geofencing_speed)
            print("Slowing for pedestrian in path!")
            if (time.time() - self.time_geofencing_last > self.geofencing_timeout):
                self.geofencing_detected = False

    def check_obstacle_detection(self):
        if self.obstacle_detected:
            speed2 = 2 * -self.max_deceleration * \
                (self.obstacle_distance - self.obstacle_distance_buffer)
            if speed2 > 0:
                obstacle_speed_req = math.sqrt(speed2)
            else:
                obstacle_speed_req = 0
            self.target_speed = min(self.target_speed, obstacle_speed_req)
            print(
                "Obstacle in {} meters, reducing max speed to {}".format(
                    self.obstacle_distance,
                    obstacle_speed_req))

    def check_poller(self):
        # send
        for socket, event in self.poller.poll(0):
            if socket is self.autoware_sock:
                self.autoware_data = messaging.recv_one(socket)
                self.target_speed = self.autoware_data.autoware.targetSpeed
                self.target_yaw_rate = self.autoware_data.autoware.targetYawRate
            elif socket is self.geofencing_sock:
                geofencing_data = messaging.recv_one(socket)
                self.geofencing_detected = geofencing_data.autowareGeofencing.pedestrianInWay
                self.time_geofencing_last = time.time()
            elif socket is self.obstacles_sock:
                obstacle_data = messaging.recv_one(socket)
                self.obstacle_detected = obstacle_data.autowareObstacles.obstacleDetection
                self.obstacle_distance = obstacle_data.autowareObstacles.obstacleDistance
            elif socket is self.autoware_cmd_sock:
                ctrl_cmd_data = messaging.recv_one(socket)
                self.target_speed = ctrl_cmd_data.autowareCommand.targetSpeed
                self.target_acceleration = ctrl_cmd_data.autowareCommand.targetAcceleration
                self.target_steering_angle = ctrl_cmd_data.autowareCommand.targetSteering

        self.CS = self.CI.update(self.CC)

        self.check_geofencing()
        self.check_obstacle_detection()

    def check_as_enabled(self):
        # Check for enable/disable signals
        # cruise_en becomes True when cruise control is activated by the driver (push on/off + lever down)
        # NOTE: cruise_en becomes True when cruise control is re-activated
        # after a fail (lever up)
        self.enabled = self.CS.cruiseState.enabled

        # Check for safety driver override
        # Currently, override is only given if the brake is pressed or the steer is actuated with sufficient torque.
        # Pressing the gas currently does not disable the autonomous system to
        # allow the driver to give a little bit more gas to the car during
        # demos (ACC like behaviour)
        if self.CS.steeringPressed or self.CS.brakePressed:  # or CS.gasPressed
            self.enabled = False

    def calculate_steering_angle(self):
        if self.use_ctrl_cmd:
            # Convert from rad to degrees
            steer_angle = math.degrees(self.target_steering_angle)
        else:
            # Using vehicle model to convert yaw rate and speed to steering
            # angle
            steer_angle = self.VM.get_steer_from_curvature(
                self.target_yaw_rate, self.current_speed)

        steer_controller_gain_normal = 110.
        steer_controller_gain_curve = 180.
        steer_controller_curve_threshold = 0.03 * (self.current_speed + 1)

        if abs(self.target_yaw_rate) < steer_controller_curve_threshold:
            steer_angle *= steer_controller_gain_normal
        else:
            steer_angle *= steer_controller_gain_curve
        steer_angle /= (2.0 * self.current_speed + 1.0)

        return steer_angle

    def command_car_interface(self, actuator_msg):
        self.CC.actuators.steer = actuator_msg.steer
        self.CC.actuators.steerAngle = actuator_msg.steerAngle
        self.CC.hudControl.visualAlert = 0
        self.CC.hudControl.setSpeed = 20
        self.CC.cruiseControl.cancel = False
        self.CC.enabled = self.enabled
        self.CI.apply(self.CC)

    def broadcast_car_state(self):
        cs_send = messaging.new_message()
        cs_send.init('carState')
        cs_send.carState = copy(self.CS)
        self.carstate.send(cs_send.to_bytes())

    def broadcast_car_control(self):
        cc_send = messaging.new_message()
        cc_send.init('carControl')
        cc_send.carControl = copy(self.CC)
        self.carcontrol.send(cc_send.to_bytes())

    def run(self):
        while is_main_thread_active():
            self.check_poller()

            actuator_msg = car.CarControl.Actuators.new_message()

            self.speed_controller()
            self.current_speed = self.CS.vEgo

            self.check_as_enabled()

            actuator_msg.steerAngle = self.calculate_steering_angle()
            actuator_msg.steer = actuator_msg.steerAngle / 360.
            ###################################################################

            print(
                "enable",
                self.enabled,
                "steer",
                actuator_msg.steer,
                "accel",
                self.CC.actuators.gas -
                self.CC.actuators.brake,
                "cur_vel",
                self.CS.vEgo,
                "cur_yaw",
                self.CS.yawRate,
                "target_yaw",
                self.target_yaw_rate,
                "target_speed",
                self.target_speed)

            self.command_car_interface(actuator_msg)

            # broadcast carState
            self.broadcast_car_state()

            # broadcast carControl
            self.broadcast_car_control()

            # Limit to 100 frames per second
            time.sleep(0.01)
            # TODO: Do some analysis on what the current refresh rate is.
            # It should be < 100Hz currently due to the processing time on top of
            # the sleep.


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Control car using autoware command data.')
    parser.add_argument(
        '--use-ctrl-cmd',
        dest='use_ctrl_cmd',
        default=True,
        help='Use autoware ctrl_cmd topic instead of twist_raw')
    args = parser.parse_args()

    autoware_controller = AutowareController(args.use_ctrl_cmd)
    autoware_controller.run()

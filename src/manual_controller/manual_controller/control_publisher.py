# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
Main ROS2 Node to generate control commands from manual user input
The message will be of aiim_autoware_msg.VehicleControlCommand type.

It does not itself interpret the data, this should be done by a child class.
"""

import time
import pygame
from aiim_autoware_msgs.msg import VehicleControlCommand
from rclpy.node import Node


class ControlPublisher(Node):
    """ Main control node class to be inherited """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('velocity_gain', 1.0),
                ('steer_gain', 1.0),
                ('command_topic', '/control_command'),
                ('update_frequency', 10.0)
            ])

        self.velocity_gain = self.get_parameter('velocity_gain').value
        self.steer_gain = self.get_parameter('steer_gain').value
        self.time_delta = 1.0 / self.get_parameter('update_frequency').value

        self.velocity_cmd = 0.0
        self.steer_cmd = 0.0

        # Create publisher
        self.publisher_ = self.create_publisher(
            VehicleControlCommand, self.get_parameter('command_topic').value, 1)

        pygame.init()

    def process_pygame_events(self):
        """ Processes the input from pygame events in child class """
        raise NotImplementedError

    def interpret_user_input(self):
        """ Converts user input to control commands """
        raise NotImplementedError

    def run(self):
        """ Start the run loop """
        # -------- Main Program Loop -----------
        while True:

            if not self.process_pygame_events():
                break

            self.interpret_user_input()

            self.publish_control_command()

            time.sleep(self.time_delta)

    def publish_control_command(self):
        """ Publishes the commands to ROS """
        msg = VehicleControlCommand()
        msg.long_accel_mps2 = self.velocity_cmd * self.velocity_gain
        msg.front_wheel_angle_rad = self.steer_cmd * self.steer_gain
        self.publisher_.publish(msg)

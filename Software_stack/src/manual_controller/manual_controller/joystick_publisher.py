# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
ROS2 node to publish control commands using joystick input
Message is of aiim_autoware_msg.VehicleControlCommand
"""

import pygame
import rclpy
from manual_controller.control_publisher import ControlPublisher


class JoystickControlPublisher(ControlPublisher):
    """ Node to handle joystick control input and publish a control message"""

    def __init__(self):
        super().__init__('joystick_control_publisher')

        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("More than one joystick found!")
        if joystick_count < 1:
            raise ValueError("No joystick found!")

        self.joystick = pygame.joystick.Joystick(0)

    def process_pygame_events(self):
        """ Processes the input from pygame events and obtains the axis values"""
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                return False

        return True

    def interpret_user_input(self):
        """ Interprets the axis values into control commands """
        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = []

        for axis in range(self.joystick.get_numaxes()):
            axes.append(self.joystick.get_axis(axis))
        # We have to flip to make the axis intuitive
        # Axis 0/1 (usually) are used for the left joystick
        self.steer_cmd = -axes[0]
        self.velocity_cmd = -axes[1]


def main(args=None):
    """ Create and run the node """
    rclpy.init(args=args)
    joystick_control_publisher = JoystickControlPublisher()
    joystick_control_publisher.run()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_control_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

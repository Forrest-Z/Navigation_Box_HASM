# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
ROS2 node to publish control commands using keyboard input
Message is of aiim_autoware_msg.VehicleControlCommand
"""

import pygame
import rclpy
from manual_controller.control_publisher import ControlPublisher


class KeyboardControlPublisher(ControlPublisher):
    """ Node to handle keyboard control input and publish a control message"""

    def __init__(self):
        super().__init__('keyboard_control_publisher')
        self.up_pressed = False
        self.down_pressed = False
        self.right_pressed = False
        self.left_pressed = False

        # Keyboard will only register within a pygame window
        pygame.display.set_mode((50, 50))

    def process_pygame_events(self):
        """ Processes the input from pygame events"""
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    self.left_pressed = True
                    print("K_LEFT pressed")
                elif event.key == pygame.K_RIGHT:
                    self.right_pressed = True
                    print("K_RIGHT pressed")
                elif event.key == pygame.K_UP:
                    self.up_pressed = True
                    print("K_UP pressed")
                elif event.key == pygame.K_DOWN:
                    self.down_pressed = True
                    print("K_DOWN pressed")
            elif event.type == pygame.KEYUP:
                print("key_released")
                if event.key == pygame.K_LEFT:
                    self.left_pressed = False
                    print("K_LEFT released")
                elif event.key == pygame.K_RIGHT:
                    self.right_pressed = False
                    print("K_RIGHT released")
                elif event.key == pygame.K_UP:
                    self.up_pressed = False
                    print("K_UP released")
                elif event.key == pygame.K_DOWN:
                    self.down_pressed = False
                    print("K_DOWN released")
        return True

    def interpret_user_input(self):
        """ Interprets the keys pressed into control commands """
        if self.up_pressed:
            self.velocity_cmd = 1.0
        elif self.down_pressed:
            self.velocity_cmd = -1.0
        else:
            self.velocity_cmd = 0.0

        if self.left_pressed:
            self.steer_cmd = 1.0
        elif self.right_pressed:
            self.steer_cmd = -1.0
        else:
            self.steer_cmd = 0.0


def main(args=None):
    """ Create and run the node """
    rclpy.init(args=args)
    keyboard_control_publisher = KeyboardControlPublisher()
    keyboard_control_publisher.run()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_control_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

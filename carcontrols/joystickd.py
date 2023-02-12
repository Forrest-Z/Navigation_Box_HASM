#!/usr/bin/env python
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# This process publishes joystick events. Such events can be suscribed by
# mocked car controller scripts.


### this process needs pygame and can't run on the EON ###

import pygame
import zmq
from selfdrive.services import service_list
import selfdrive.messaging as messaging
from common.numpy_fast import clip


def joystick_thread():
    pygame.init()

    context = zmq.Context()
    command_sock = messaging.pub_sock(
        context, service_list['udpInMsg'].port)
    joystick_sock = messaging.pub_sock(
        context, service_list['testJoystick'].port)

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()

    # Initialize the joysticks
    pygame.joystick.init()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count > 1:
        raise ValueError("More than one joystick attached")
    elif joystick_count < 1:
        raise ValueError("No joystick found")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # -------- Main Program Loop -----------
    while True:
        # EVENT PROCESSING STEP
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                pass
            # Available joystick events: JOYAXISMOTION JOYBALLMOTION
            # JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = []
        buttons = []

        for a in range(joystick.get_numaxes()):
            axes.append(joystick.get_axis(a))

        for b in range(joystick.get_numbuttons()):
            buttons.append(joystick.get_button(b))

        accel_axis = clip(-axes[1] * 1.05, -1., 1.)
        steer_axis = clip(-axes[3] * 1.05, -1., 1.)

        command_msg = messaging.new_message()
        command_msg.init('udpInMsg')
        command_msg.udpInMsg.accel = accel_axis
        command_msg.udpInMsg.steer = steer_axis
        command_sock.send(command_msg.to_bytes())

        joystick_msg = messaging.new_message()
        joystick_msg.init('testJoystick')
        joystick_msg.testJoystick.axes = axes
        joystick_msg.testJoystick.buttons = map(bool, buttons)
        joystick_sock.send(joystick_msg.to_bytes())

        # Limit to 100 frames per second
        # clock.tick(100)
        clock.tick(100)


if __name__ == "__main__":
    joystick_thread()

#!/usr/bin/env python
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
This process links the threads for controlling the car through the xbox controller together in a single thread
These threads are set up as deamons so that they are killed when the main thread is killed
"""

from joystickd import joystick_thread
from debug_controls import control_thread
from threading import Thread
import time


def main_thread():
    t_joystick = Thread(target=joystick_thread)
    t_control = Thread(target=control_thread)
    t_joystick.setDaemon(True)
    t_control.setDaemon(True)
    t_joystick.start()
    t_control.start()

    while(t_joystick.is_alive() and t_control.is_alive()):
        time.sleep(0.1)


if __name__ == "__main__":
    main_thread()

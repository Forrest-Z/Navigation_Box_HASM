#!/usr/bin/env python

# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
This process links the threads for controlling the car through the udp server together through a single thread
These threads are set up as deamons so that they are killed when the main thread is killed
"""

from ros2_udpd import UdpConnection
from ros2_controls import UdpController
import threading
import time


def main_thread():
    udp_connection = UdpConnection()
    udp_controller = UdpController()
    t_udp = threading.Thread(target=udp_connection.run)
    t_control = threading.Thread(target=udp_controller.run)
    t_udp.setDaemon(True)
    t_control.setDaemon(True)
    t_udp.start()
    t_control.start()

    while(t_udp.is_alive() and t_control.is_alive()):
        time.sleep(0.1)


if __name__ == "__main__":
    main_thread()

#!/usr/bin/env python
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
This process links the threads for controlling the car through Autoware together in one main thread
These threads are set up as deamons so that they are killed when the main thread is killed
"""

from autowared import AutowareNode
from autoware_controls import AutowareController
import threading
import time


def main_thread():
    autoware_node = AutowareNode()
    autoware_controller = AutowareController()
    t_control = threading.Thread(target=autoware_controller.run)
    t_control.setDaemon(True)
    t_control.start()
    t_autoware = threading.Thread(target=autoware_node.run)
    t_autoware.setDaemon(True)
    t_autoware.start()

    while(t_control.is_alive() and t_autoware.is_alive()):
        time.sleep(0.1)


if __name__ == "__main__":
    main_thread()

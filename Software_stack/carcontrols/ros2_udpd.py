#!/usr/bin/env python

# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
This process receives control messages for the car over an udp server
"""

# Openpilot imports
import zmq
from selfdrive.services import service_list
import selfdrive.messaging as messaging
import socket as inet_socket

# udp imports
import msgpack
import select

import time


# UDP_IP = "192.168.1.113"
UDP_IP = "192.168.2.45"
# UDP_IP = "192.168.8.255"
UDP_PORT_IN = 5016
SOCKET_TIMEOUT = 1.0  # seconds
# SOCKET_BUFFER_SIZE = 1024  # bytes
SOCKET_BUFFER_SIZE = 128  # bytes


class UdpConnection:
    def __init__(self):
        self.init_openpilot_messaging()

        self.eventid = 0

        # Open sockets
        self.udp_sock = inet_socket.socket(inet_socket.AF_INET,  # Internet
                                           inet_socket.SOCK_DGRAM)  # UDP
        self.udp_sock.setsockopt(
            inet_socket.SOL_SOCKET,
            inet_socket.SO_SNDBUF,
            SOCKET_BUFFER_SIZE)
        self.udp_sock.bind((UDP_IP, UDP_PORT_IN))
        print("Connected to {}:{}".format(UDP_IP, UDP_PORT_IN))

    def init_openpilot_messaging(self):
        self.op_context = zmq.Context()
        self.op_poller = zmq.Poller()

        self.command_pub = messaging.pub_sock(
            self.op_context, service_list['udpInMsg'].port)

    def check_if_data_available(self):
        return select.select([self.udp_sock], [], [], SOCKET_TIMEOUT)[0]

    def run(self):
        while True:
            # Check if data is ready
            data_ready = self.check_if_data_available()
            if data_ready:
                udp_data = self.udp_sock.recv(SOCKET_BUFFER_SIZE)

                # unpackb is current standard for msgpack-python, use unpack
                # for legacy
                [target_accel,
                    target_steer] = msgpack.unpackb(udp_data, raw=False)

                # Forward message to vehicle controller
                command_msg = messaging.new_message()
                command_msg.init('udpInMsg')
                command_msg.udpInMsg.accel = target_accel
                command_msg.udpInMsg.steer = target_steer
                self.command_pub.send(command_msg.to_bytes())
                # print([target_accel,
                #     target_steer])

                # Check and update and log status if changed
                # if not self.external_connected:
                #     print("ROS2 Connected!")
                #     self.external_connected = True

            # else:
            #     # We did not receive, update and log status
            #     if self.external_connected:
            #         print("ROS2 Disconnected!")
            #         self.external_connected = False

            # Limit to 100 frames per second
            time.sleep(0.01)


if __name__ == "__main__":
    udp_connection = UdpConnection()
    udp_connection.run()

"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""

"""
ROS2 node which subscribe to aiim_control_msg.VehicleControlCommand type 
and sends desired acc and steering wheel angle
to the udp server udpd.py of AIIM Prius for controlling the car.
"""
import math
import socket
import msgpack

import rclpy
from rclpy.node import Node

from aiim_autoware_msgs.msg import VehicleControlCommand

SOCKET_TIMEOUT = 1.0  # seconds
SOCKET_BUFFER_SIZE = 1024  # bytes


def wheel_angle_to_steering_angle(wheel_angle):
    """
    It converts wheel angle to steering wheel angle
    """
    wheel_angle_degrees = math.degrees(wheel_angle)

    # steering ratio of AIIM Toyota Prius should be 19:1:1
    # TODO (vehicles team): need to test this
    steering_angle_degrees = wheel_angle_degrees * 19.
    return steering_angle_degrees


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('server_ip_address', None),
                ('server_ip_port', None),
                ('topic', None)
            ])
        self.ip_address_srv = self.get_parameter('server_ip_address').\
            get_parameter_value().string_value
        self.ip_port_out = self.get_parameter('server_ip_port').\
            get_parameter_value().integer_value
        self.topic_in = self.get_parameter('topic').get_parameter_value().string_value

        self.udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.udp_socket.connect((self.ip_address_srv, self.ip_port_out))
        self.get_logger().debug("connected ")

        # TODO (davide.congiu):
        #  test if create_subscription can be not assigned to subscription attribute
        self.subscription = self.create_subscription(
            VehicleControlCommand,
            self.topic_in,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        desired_acc = msg.long_accel_mps2
        desired_wheel_angle = wheel_angle_to_steering_angle(msg.front_wheel_angle_rad)
        packet_body = msgpack.packb([desired_acc, desired_wheel_angle])

        # send control command via udp message
        self.udp_socket.sendto(packet_body, (self.ip_address_srv, self.ip_port_out))
        self.get_logger().debug('control message forwarded to {}:{}'.
                                format(self.ip_address_srv, self.ip_port_out))


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

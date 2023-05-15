"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    name = 'udp_control_msg_client'
    share_dir = get_package_share_directory(name)
    parameter_file = os.path.join(share_dir, 'params', 'default.yaml')
    node = Node(package=name,
                namespace='/',
                executable=name,
                name=name,
                output='screen',
                # Known issue: can prevent not showing output when run as a sub process.
                # emulate_tty=True,
                parameters=[parameter_file],
                )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld

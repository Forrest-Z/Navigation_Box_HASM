"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""
from launch import LaunchDescription
from aiim_rospy.launch_templates import generate_node_action


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(generate_node_action(
        package_name='collision_avoidance_client',
        node_name='collision_avoidance_client',
        params_file='default.yaml'))
    return ld
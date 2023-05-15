"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""

import aiim_rospy.launch_templates as aiim_lt
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration


def add_pure_pursuit_actions(
        launch_description,
        params_package="",
        params_filename=""):
    """ Add nodes required for control using PurePursuit. """
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='pure_pursuit_nodes',
            node_name='pure_pursuit_node_exe',
            params_file=params_filename,
            params_package=params_package,
            condition=UnlessCondition(LaunchConfiguration('record_trajectory'))
        )
    )

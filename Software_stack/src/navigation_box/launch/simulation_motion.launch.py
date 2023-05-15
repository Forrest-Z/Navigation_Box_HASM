"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""
import aiim_rospy.launch_templates as aiim_lt
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
import navigation_box.navigation_box as navbox


def generate_launch_description():
    """ Adds all actions for the simulation of the navigation box. """
    launch_description = navbox.create_common_launch_description()

    # Simulation
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='agv_simulator',
            node_name='agv_simulator',
            params_file='mmp_upstairs.yaml',
            params_package='navigation_box')
    )
    # Control
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='manual_controller',
            node_name='keyboard_publisher',
            params_file='mmp_upstairs.yaml',
            params_package='navigation_box',
            condition=UnlessCondition(
                LaunchConfiguration('replay_trajectory'))))
    # Visual
    aiim_lt.add_urdf_model_action(
        launch_description,
        urdf_package='navigation_box',
        urdf_file='agv.urdf.xml')

    return launch_description

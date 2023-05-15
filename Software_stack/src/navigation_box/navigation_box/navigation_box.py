"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""
import aiim_rospy.launch_templates as aiim_lt
import navigation_box.control_nodes as control
import navigation_box.planning_nodes as planning
from launch import LaunchDescription


def create_common_launch_description(launch_description):
    """ Adds core actions for the navigation box. """
    # launch_description = LaunchDescription()
    
    # System Calibration
    launch_description.add_action(
        aiim_lt.generate_static_tf_identity(
            frame_from='base_link',
            frame_to='pcap_streamer_frame'))
    # Planning
    planning.add_recordreplay_actions(
        launch_description,
        trajectory_file="/home/carpc/Software/BuurAutonoom/navigation_box/src/mqtt_pubsub/data/15_P4P1parkingSimpleDemoTrial3.csv", 
        params_package="navigation_box",
        params_filename="tuecampus.yaml")
    # Control
    control.add_pure_pursuit_actions(
        launch_description,
        params_package="navigation_box",
        params_filename="tuecampus.yaml")
    # To visualize using rviz:
    aiim_lt.add_rviz_run_action(
        launch_description,
        rviz_package="navigation_box",
        rviz_file="navigation_box.rviz")
    return launch_description

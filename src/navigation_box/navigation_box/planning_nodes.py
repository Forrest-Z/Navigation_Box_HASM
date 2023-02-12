"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""

import aiim_rospy.launch_templates as aiim_lt
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration


def add_record_action_publish(trajectory_file):
    """ Adds an action request to record a trajectory. """
    return ExecuteProcess(
        cmd=[
            'ros2',
            'action',
            'send_goal',
            '/recordtrajectory',
            'recordreplay_planner_actions/action/RecordTrajectory',
            f'{{record_path: {trajectory_file}}}'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_trajectory'))
    )


def add_replay_action_publish(trajectory_file):
    """ Adds an action request to replay a trajectory. """
    return ExecuteProcess(
        cmd=[
            'ros2',
            'action',
            'send_goal',
            '/replaytrajectory',
            'recordreplay_planner_actions/action/ReplayTrajectory',
            f'{{replay_path: {trajectory_file}}}'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('replay_trajectory'))
    )


def add_recordreplay_launch_arguments(launch_description):
    """ This adds an argument to toggle replay or record. """
    launch_description.add_action(DeclareLaunchArgument(
        'replay_trajectory',
        default_value='False',
        description='Replay the given trajectory file'
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'record_trajectory',
        default_value='False',
        description='Record to the given trajectory file'
    ))


def add_recordreplay_actions(
        launch_description,
        trajectory_file,
        params_package="",
        params_filename=""):
    """ Add nodes required for planning using recordreplay, if autoware planners are disabeled """
    # Add argument
    add_recordreplay_launch_arguments(launch_description)
    # Add recordreplay_planner
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='recordreplay_planner_nodes',
            node_name='recordreplay_planner_node_exe',
            params_file=params_filename,
            params_package=params_package,
            condition=UnlessCondition(LaunchConfiguration('use_autoware_planner'))))
    # Add conditional action requests
    launch_description.add_action(add_replay_action_publish(trajectory_file))
    launch_description.add_action(add_record_action_publish(trajectory_file))


def add_collision_avoidance_actions(
        launch_description,
        params_package="",
        params_filename=""):
    """ Add nodes required for updating trajectory with collision avoidance. """
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='path_cluster_detection',
            params_file=params_filename,
            params_package=params_package))
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='object_collision_estimator_nodes',
            node_name='object_collision_estimator_node_exe',
            params_file=params_filename,
            params_package=params_package))
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='collision_avoidance_client',
            node_name='collision_avoidance_client',
            params_file=params_filename,
            params_package=params_package,
            condition=UnlessCondition(LaunchConfiguration('use_autoware_planner'))))

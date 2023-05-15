"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""

__all__ = [
    'generate_node_action',
    'generate_rviz_run_action',
    'generate_static_tf_identity',
    'generate_static_tf_euler',
    'generate_static_tf_quat']

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_node_action(
        package_name,
        node_name="",
        params_file="default.yaml",
        params_package="",
        output="screen",
        condition=None):
    """ Generates an action to launch a ROS node. """
    if params_package == "":
        params_package = package_name
    if node_name == "":
        node_name = package_name
    share_dir = get_package_share_directory(params_package)
    parameter_file = os.path.join(share_dir, 'params', params_file)
    return Node(package=package_name,
                namespace='/',
                executable=node_name,
                name=node_name,
                # Can output stdout/stderr to log or screen(console)
                output={'both': f'{output}'},
                # Known issue: can prevent not showing output when run as a sub process.
                # emulate_tty=True,
                parameters=[parameter_file],
                condition=condition
                )


def add_rviz_run_action(launch_description, rviz_package="", rviz_file=""):
    """ Adds an action to launch rviz to the launch_description. """
    # By default it will not give any arguments and thus assume default system
    # rviz config
    file_args = []
    # But, if both a package and filename are given, we will switch to that
    # instead
    if rviz_package and rviz_file:
        share_dir = get_package_share_directory(rviz_package)
        file_args = ['-d', os.path.join(share_dir, 'rviz', rviz_file)]
    # Add launch argument
    launch_description.add_action(DeclareLaunchArgument(
        'visualize',
        default_value='False',
        description='Visualize using rviz'
    ))
    launch_description.add_action(Node(package='rviz2',
                                       namespace='/',
                                       executable='rviz2',
                                       name='rviz2',
                                       # This redirects all rviz spam to log
                                       output={'both': 'log'},
                                       arguments=file_args,
                                       condition=IfCondition(
                                           LaunchConfiguration('visualize'))
                                       ))


def add_urdf_model_action(launch_description, urdf_package="", urdf_file=""):
    launch_description.add_action(DeclareLaunchArgument(
        'show_vehicle_model',
        default_value='False',
        description='Show vehicle model in rviz'
    ))
    """ Publish the robot model to be visualized in rviz. """
    urdf = os.path.join(
        get_package_share_directory(urdf_package),
        'models',
        urdf_file)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    launch_description.add_action(Node(package='robot_state_publisher',
                                       namespace='/',
                                       executable='robot_state_publisher',
                                       name='robot_state_publisher',
                                       # This redirects all spam to log
                                       output={'both': 'log'},
                                       parameters=[
                                           {'robot_description': robot_desc}
                                       ],
                                       arguments=[urdf],
                                       condition=IfCondition(
                                           LaunchConfiguration('show_vehicle_model'))
                                       ))


def generate_static_tf_identity(
        frame_from,
        frame_to,
        output='log'):
    """ Generates an action to publish an identity transform between two frames. """
    return generate_static_tf_euler(
        frame_from=frame_from,
        frame_to=frame_to,
        output=output)


def generate_static_tf_euler(
        frame_from,
        frame_to,
        dx=0.0,
        dy=0.0,
        dz=0.0,
        droll=0.0,
        dyaw=0.0,
        dpitch=0.0,
        output='log'):
    """ Generates an action to publish a transform between two frames in euler coordinates. """
    return Node(
        package='tf2_ros',
        namespace='/',
        executable='static_transform_publisher',
        name=f'{frame_from}_to_{frame_to}',
        output=output,
        arguments=[
            f'{dx}',
            f'{dy}',
            f'{dz}',
            f'{dyaw}',
            f'{dpitch}',
            f'{droll}',
            frame_from,
            frame_to])


def generate_static_tf_quat(
        frame_from,
        frame_to,
        dx=0.0,
        dy=0.0,
        dz=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=0.0,
        output='log'):
    """ Generates an action to publish a transform between two frames in quaternion coordinates. """
    return Node(
        package='tf2_ros',
        namespace='/',
        executable='static_transform_publisher',
        name=f'{frame_from}_to_{frame_to}',
        output=f'{output}',
        arguments=[
            f'{dx}',
            f'{dy}',
            f'{dz}',
            f'{qx}',
            f'{qy}',
            f'{qz}',
            f'{qw}',
            frame_from,
            frame_to])


def generate_rosbag_record_action(topics="", output_file=""):
    """ Generates an action to record a ROSbag. """
    # Define topics to record
    # If no topics are given, we record all (-a)
    if topics == "":
        topics = "-a"
    # If output file is given, we have to add -o
    if output_file != "":
        output = ['-o', output_file]
    else:
        output = ""
    return ExecuteProcess(
        cmd=['ros2', 'bag', 'record', topics, output],
        output='screen'
    )


def generate_rosbag_play_action(filename, condition=None):
    """ Generates an action to play a rosbag. """
    return ExecuteProcess(
        cmd=['ros2', 'bag', 'play', filename],
        output='screen',
        condition=condition
    )

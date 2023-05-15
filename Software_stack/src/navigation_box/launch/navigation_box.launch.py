"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""
import aiim_rospy.launch_templates as aiim_lt
import navigation_box.localization_nodes as localization
import navigation_box.planning_nodes as planning
import navigation_box.navigation_box as navbox
import navigation_box.map_loader as map
import navigation_box.path_planners as pathPlanning
import navigation_box.mqtt_interface as mqttInterface
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch import LaunchDescription


def add_data_input_actions(launch_description):
    """ Adds the actions related to data input. """
    # Launch arguments
    launch_description.add_action(DeclareLaunchArgument(
        'use_rosbag',
        default_value='False',
        description='Use data from rosbag as input'
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'use_pcap_streamer',
        default_value='False',
        description='Use pcap streamer data as input'
    ))
    
    
    # Either use replay from rosbag:
    launch_description.add_action(DeclareLaunchArgument(
        'rosbag_path',
        description='Location of the rosbag path',
        condition=IfCondition(LaunchConfiguration('use_rosbag'))
    ))
    launch_description.add_action(
        aiim_lt.generate_rosbag_play_action(
            filename=LaunchConfiguration('rosbag_path'),
            condition=IfCondition(LaunchConfiguration('use_rosbag'))))
    
    # Stream live:
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='pcap_streamer',
            node_name='pcap_streamer',
            params_file='PCDirectories.yaml',
            params_package='navigation_box',
            condition=IfCondition(LaunchConfiguration('use_pcap_streamer'))))

def declare_buurAutonoom_launchArguments(launch_description):
    launch_description.add_action(DeclareLaunchArgument(
        'use_autoware_planner',
        default_value='False',
        description='True -> Use autoware planners for route planning, False -> Use recordReplay node for route loading'
    ))
    launch_description.add_action(DeclareLaunchArgument(
        'use_mqtt_interface',
        default_value='False',
        description='True -> Use the mqtt interface node'
    ))



def add_vehicle_interface_actions(launch_description):
    launch_description.add_action(DeclareLaunchArgument(
        'output_udp',
        default_value='False',
        description='Communicate vehicle command through udp.'
    ))
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='udp_control_msg_client',
            params_package="navigation_box",
            params_file="tuecampus.yaml",
            condition=IfCondition(LaunchConfiguration('output_udp'))))

def add_buurautonoom_launch(launch_description):
    navigation_box_pkg_prefix = get_package_share_directory('navigation_box')
    launch_description.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_box_pkg_prefix, '/launch/buurautonoom.launch.py']),
        launch_arguments={}.items()
    ))

def generate_launch_description():
    """ Adds all actions for the navigation box. """
    # Create launch description 
    launch_description = LaunchDescription()

    # Declare all buurAutonoom launch arguments
    declare_buurAutonoom_launchArguments(launch_description)

    # Core
    navbox.create_common_launch_description(launch_description)
    
    # Input
    add_data_input_actions(launch_description)
    
    # Localization
    localization.add_ndt_actions(launch_description,
                                 params_package="navigation_box",
                                 params_filename="tuecampus.yaml")
    
    # Collision avoidance
    planning.add_collision_avoidance_actions(
        launch_description,
        params_package="navigation_box",
        params_filename="tuecampus.yaml")
    
    # To Vehicle
    add_vehicle_interface_actions(launch_description)

    # Launch Map loader used for loading lanelet2 maps based on "use_autoware_planner" launch arguments
    map.add_map_loader_action(launch_description, condition = IfCondition(LaunchConfiguration('use_autoware_planner')))
    
    # Launch Autoware route planners based on "use_autoware_planner" launch arguments
    pathPlanning.add_path_planners_action(launch_description, condition = IfCondition(LaunchConfiguration('use_autoware_planner')))
    
    # Launch MQTT Interface node based on "use_mqtt_interface" launch arguments
    mqttInterface.add_mqtt_interface_action(launch_description, condition = IfCondition(LaunchConfiguration('use_mqtt_interface')), params_package="navigation_box" ,params_file="tuecampus.yaml")
     

    return launch_description

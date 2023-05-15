
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def add_map_loader_action(
        launch_description, condition
        ):
    
    lanelet2_map_provider_pkg_prefix = get_package_share_directory('lanelet2_map_provider')
        
    lanelet2_map_provider_param_file = os.path.join(
    lanelet2_map_provider_pkg_prefix, 'param/lanelet2_map_provider.param.yaml')
    
    map_osm_file = os.path.join(
        lanelet2_map_provider_pkg_prefix, 'data/autonomoustuff_parking_lot.osm')
    
    # lanelet2_map_provider_param = DeclareLaunchArgument(
    #     'lanelet2_map_provider_param_file',
    #     default_value=lanelet2_map_provider_param_file,
    #     description='Path to parameter file for Lanelet2 Map Provider'
    # )
    output="screen"

    launch_description.add_action(
        Node(package='lanelet2_map_provider',
            namespace='had_maps',
            executable='lanelet2_map_provider_exe',
            name='lanelet2_map_provider_node',
            # Can output stdout/stderr to log or screen(console)
            output={'both': f'{output}'},
            parameters=[lanelet2_map_provider_param_file, {'map_osm_file': map_osm_file}],
            condition=condition
        )
    )
    launch_description.add_action(
        Node(package='lanelet2_map_provider',
            namespace='had_maps',
            executable='lanelet2_map_visualizer_exe',
            name='lanelet2_map_visualizer_node',
            # Can output stdout/stderr to log or screen(console)
            output={'both': f'{output}'},
            condition=condition
        )
    )

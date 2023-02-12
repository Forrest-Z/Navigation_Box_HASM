#import aiim_rospy.launch_templates as aiim_lt
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    lanelet2_map_provider_pkg_prefix = get_package_share_directory('lanelet2_map_provider')
        
    lanelet2_map_provider_param_file = os.path.join(
    lanelet2_map_provider_pkg_prefix, 'param/lanelet2_map_provider.param.yaml')
    
    map_osm_file = os.path.join(
        lanelet2_map_provider_pkg_prefix, 'data/autonomoustuff_parking_lot.osm')
    
    
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    
    #Nodes
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file'),
                    {'map_osm_file': map_osm_file}]
    )
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps'
    )





    behavior_planner_pkg_prefix = get_package_share_directory('behavior_planner_nodes')

    freespace_planner_pkg_prefix = get_package_share_directory('freespace_planner_nodes')

    costmap_generator_pkg_prefix = get_package_share_directory('costmap_generator_nodes')
    
    lane_planner_pkg_prefix = get_package_share_directory('lane_planner_nodes')

    behavior_planner_param_file = os.path.join(
        behavior_planner_pkg_prefix, 'param/behavior_planner.param.yaml')

    vehicle_characteristics_param_file = os.path.join(
        behavior_planner_pkg_prefix, 'param/vehicle_characteristics.param.yaml')    

    freespace_planner_param_file = os.path.join(
        freespace_planner_pkg_prefix, 'param/freespace_planner.param.yaml')

    costmap_generator_param_file = os.path.join(
        costmap_generator_pkg_prefix, 'param/costmap_generator.param.yaml')
    
    vehicle_constants_manager_param_file = os.path.join(
        freespace_planner_pkg_prefix, 'param/lexus_rx_hybrid_2016.param.yaml')

    lane_planner_param_file = os.path.join(
        lane_planner_pkg_prefix, 'param/lane_planner.param.yaml')
    
    
    
    lane_planner_param = DeclareLaunchArgument(
        'lane_planner_param_file',
        default_value=lane_planner_param_file,
        description='Path to parameter file for lane planner'
    )

    freespace_planner_param = DeclareLaunchArgument(
        'freespace_planner_param_file',
        default_value=freespace_planner_param_file,
        description='Path to parameter file for freespace_planner'
    )

    costmap_generator_param = DeclareLaunchArgument(
        'costmap_generator_param_file',
        default_value=costmap_generator_param_file,
        description='Path to parameter file for costmap generator'
    )

    vehicle_constants_manager_param = DeclareLaunchArgument(
        'vehicle_constants_manager_param_file',
        default_value=vehicle_constants_manager_param_file,
        description='Path to parameter file for vehicle_constants_manager'
    )
    
    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=behavior_planner_param_file,
        description='Path to parameter file for behavior planner'
    )

    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='True',
        description='Enable obstacle detection'
    )

    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )

    global_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )

    behavior_planner = Node(
        package='behavior_planner_nodes',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        parameters=[
            LaunchConfiguration('behavior_planner_param_file'),
            {'enable_object_collision_estimator': LaunchConfiguration('with_obstacles')},
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('gear_report', '/vehicle/gear_report'),
            ('gear_command', '/vehicle/gear_command')
        ]
    )


    freespace_planner = Node(
        package='freespace_planner_nodes',
        executable='freespace_planner_node_exe',
        name='freespace_planner',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('freespace_planner_param_file'),
            LaunchConfiguration('vehicle_constants_manager_param_file')
        ]
    )

    costmap_generator = Node(
        package='costmap_generator_nodes',
        executable='costmap_generator_node_exe',
        name='costmap_generator_node',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('costmap_generator_param_file'),
        ],
        remappings=[
            ('~/client/HAD_Map_Service', '/had_maps/HAD_Map_Service')
        ]
    )

    lane_planner = Node(
        package='lane_planner_nodes',
        name='lane_planner_node',
        namespace='planning',
        executable='lane_planner_node_exe',
        parameters=[
            LaunchConfiguration('lane_planner_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )



    return LaunchDescription([
        costmap_generator_param,
        freespace_planner_param,
        lane_planner_param,
        vehicle_constants_manager_param,
        vehicle_characteristics_param,
        with_obstacles_param,
        behavior_planner_param,
        behavior_planner,
        global_planner,
        costmap_generator,
        freespace_planner,
        lane_planner
        # lanelet2_map_provider_param,
        # lanelet2_map_provider,
        # lanelet2_map_visualizer
    ])

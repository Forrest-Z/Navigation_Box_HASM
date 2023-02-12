
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os


def add_path_planners_action(
        launch_description, condition
        ):

    # region Getting the packages paths

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

    # endregion

    # region Declaring Launch arguments

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
    # endregion

    # region Creating the nodes

    global_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')],
        condition=condition
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
        ],
        condition=condition
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
        ],
        condition=condition
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
        ],
        condition=condition
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
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')],
        condition=condition
    )
    # endregion

    # region Adding actions to launch_description

    launch_description.add_action(costmap_generator_param)
    launch_description.add_action(freespace_planner_param)
    launch_description.add_action(lane_planner_param)
    launch_description.add_action(vehicle_constants_manager_param)
    launch_description.add_action(vehicle_characteristics_param)
    launch_description.add_action(with_obstacles_param)
    launch_description.add_action(behavior_planner_param)
    launch_description.add_action(behavior_planner)
    launch_description.add_action(global_planner)
    launch_description.add_action(costmap_generator)
    launch_description.add_action(freespace_planner)
    launch_description.add_action(lane_planner)

    #endregion


"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""

import aiim_rospy.launch_templates as aiim_lt


def add_ndt_actions(
        launch_description,
        params_package="",
        params_filename=""):
    """ Add nodes required for localization. """
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='points_map_loader',
            params_file='PCDirectories.yaml',
            params_package='navigation_box'))
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='voxel_grid_filter',
            params_file=params_filename,
            params_package=params_package))
    launch_description.add_action(
        aiim_lt.generate_node_action(
            package_name='ndt_localization',
            params_file=params_filename,
            params_package=params_package,
            output='log'))
    # This can be used to manually align the initial point cloud with the map.
    # You should disable the ndt_localization action.
    # launch_description.add_action(
    #     aiim_lt.generate_static_tf_euler(
    #         frame_from='map',
    #         frame_to='base_link',
    #         dx=29.2,
    #         dy=21.7,
    #         dz=3.5,
    #         dyaw=0.1))

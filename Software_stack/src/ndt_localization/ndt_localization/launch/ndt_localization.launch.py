#  Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
#  Information classification: Confidential
#  This content is protected by international copyright laws.
#  Reproduction and distribution is prohibited without written permission.

from aiim_rospy.launch_templates import generate_node_action
from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        generate_node_action(
            name='ndt_localization',
            params_file='tuecampus.yaml'))
    return ld

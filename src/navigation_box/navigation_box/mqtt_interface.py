
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def add_mqtt_interface_action(
        launch_description, condition, params_package="navigation_box", params_file="tuecampus.yaml"
        ):
            
    output="screen"
    share_dir = get_package_share_directory(params_package)
    parameter_file = os.path.join(share_dir, 'params', params_file)
    launch_description.add_action(
        Node(package='mqtt_pubsub',
            namespace='',
            executable='mqtt_autoware_interface',
            name='mqtt_autoware_interface',
            # Can output stdout/stderr to log or screen(console)
            output={'both': f'{output}'},
            parameters=[parameter_file],
            condition=condition,
            prefix = ['gnome-terminal --profile=Hold -- '],
        )
    )


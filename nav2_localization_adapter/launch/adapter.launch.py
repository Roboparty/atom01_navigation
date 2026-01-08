import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_localization_adapter')
    config_file = os.path.join(pkg_share, 'config', 'adapter_params.yaml')

    adapter_node = Node(
        package='nav2_localization_adapter',
        executable='nav2_localization_adapter_node',
        name='nav2_localization_adapter',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        adapter_node,
    ])

"""
多点导航(Waypoint Navigation) Launch文件

Usage:
  ros2 launch nav2_localization_adapter waypoint_navigation.launch.py

  # 使用自定义航点
  ros2 launch nav2_localization_adapter waypoint_navigation.launch.py \
      waypoints_file:=/path/to/custom_waypoints.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    adapter_pkg = get_package_share_directory('nav2_localization_adapter')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints_file')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(adapter_pkg, 'config', 'nav2_params_dwb_garden.yaml'),
        description='Full path to Nav2 params file'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(adapter_pkg, 'map', 'garden.yaml'),
        description='Full path to map yaml file'
    )
    
    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(adapter_pkg, 'config', 'waypoints.yaml'),
        description='Full path to waypoints yaml file'
    )

    # Include the main navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(adapter_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_file,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map_file,
        declare_waypoints_file,
        navigation_launch,
    ])

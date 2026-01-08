"""
Navigation launch file integrating robots_localization with Nav2

Usage:
  ros2 launch nav2_localization_adapter navigation.launch.py \
      map:=/path/to/map.yaml \
      params_file:=/path/to/nav2_params.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    adapter_pkg = get_package_share_directory('nav2_localization_adapter')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(adapter_pkg, 'config', 'nav2_params_mppi_garden.yaml'),
        description='Full path to Nav2 params file'
    )
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(adapter_pkg, 'map', 'garden.yaml'),
        description='Full path to map yaml file'
    )

    # Localization adapter node - bridges robots_localization to Nav2
    adapter_config = os.path.join(adapter_pkg, 'config', 'adapter_params.yaml')
    adapter_node = Node(
        package='nav2_localization_adapter',
        executable='nav2_localization_adapter_node',
        name='nav2_localization_adapter',
        output='screen',
        parameters=[adapter_config, {'use_sim_time': use_sim_time}],
    )

    # Nav2 navigation stack (without AMCL - we use our own localization)
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    # Map server (lifecycle managed together with nav stack)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
    )

    # Lifecycle manager for map server (ensure map is activated)
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    # Static TF: world -> map (identity)
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map_file,
        map_server_node,
        map_lifecycle_manager,
        world_to_map_tf,
        adapter_node,
        nav2_navigation,
    ])

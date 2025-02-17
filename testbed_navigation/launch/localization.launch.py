import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_file_path = os.path.join(
        get_package_share_directory('testbed_bringup'),
        'maps',
        'testbed_world.yaml'
    )

    params_file_path = os.path.join(
        get_package_share_directory('testbed_navigation'),
        'config',
        'amcl_params.yaml' )
    return LaunchDescription([

        LogInfo(msg="Launching AMCL Localization..."),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}]
        ),
        
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'yaml_filename': params_file_path}],
            remappings=[('/scan', '/scan')]  # Replace with the appropriate scan topic if needed
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # Change to True if using simulation time
                'autostart': True,     # Automatically configure and activate lifecycle nodes
                'node_names': ['map_server']  # List of nodes to manage
            }]
        )
    ])

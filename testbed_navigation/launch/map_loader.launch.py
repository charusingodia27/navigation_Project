from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve the absolute path to the YAML file in testbed_bringup
    map_file_path = os.path.join(
        get_package_share_directory('testbed_bringup'),
        'maps',
        'testbed_world.yaml'
    )

    return LaunchDescription([
        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file_path
            }]
        ),
        
        # Lifecycle Manager Node
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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Dynamically get the map file path
    map_file_path = os.path.join(
        get_package_share_directory('testbed_bringup'),
        'maps',
        'testbed_world.yaml'
    )

    return LaunchDescription([
        # Declare arguments for map and localization
        DeclareLaunchArgument('map', default_value=map_file_path),
        DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),

        # Include testbed_full_bringup launch file
        IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory('testbed_bringup'),
                'launch',
                'testbed_full_bringup.launch.py'
            )
        ),
        
        # Launch map server using nav2_map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}],
            remappings=[('/map', 'map')]
        ),
        
        # Launch AMCL for localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('testbed_navigation'), 'config', 'amcl_params.yaml'), LaunchConfiguration('use_sim_time')],
        ),
        
        # Launch the planner and controller
        Node(
            package='nav2_bringup',
            executable='nav2_planner',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('testbed_navigation'), 'config', 'nav2_params.yaml')],
        ),
        
        # Launch the behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('testbed_navigation'), 'config', 'nav2_params.yaml')],
        ),

        # Collision monitor (optional)
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('testbed_navigation'), 'config', 'nav2_params.yaml')],
        ),
        
        # Velocity smoother (optional)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('testbed_navigation'), 'config', 'nav2_params.yaml')],
        ),
        
        # Test goal setting (send test goal)
        Node(
            package='rclcpp',
            executable='test_goal_sender',
            name='test_goal_sender',
            output='screen',
        ),
    ])


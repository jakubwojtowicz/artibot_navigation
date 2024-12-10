#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('artibot_navigation')
    nav2_config_path = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'slam_mode': True,              # Włączenie trybu SLAM
                'base_frame': 'base_link',      # Główna ramka robota
                'odom_frame': 'odom',           # Ramka odometrii
                'map_frame': 'map',             # Ramka mapy
                'scan_topic': '/scan',          # Temat skanów lidarowych
                'use_scan_matching': True       # Użycie dopasowywania skanów
            }]
        ),
        # AMCL (zbędny w przypadku SLAM, ale pozostawiony w razie potrzeby lokalizacji po zapisaniu mapy)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config_path]
        ),
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config_path]
        ),
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config_path]
        ),
        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config_path]
        ),
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'slam_toolbox',  # Dodano SLAM Toolbox
                    'planner_server',
                    'controller_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])


#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    def_config_file = os.path.join(
        get_package_share_directory('artibot_navigation'),
        'config',
        'artibot_navigation.yaml'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=def_config_file,
        description='Path to the configuration file for the nodes.'
    )
    config_file = LaunchConfiguration('config_file')

    collision_avoidance_planner_node=Node(
            package='artibot_navigation',
            executable='collision_avoidance_planner_node.py',
            name='collision_avoidance_planner_node',
            parameters=[config_file],
            output='screen'
        )
    exploration_planner_node=Node(
            package='artibot_navigation',
            executable='exploration_planner_node.py',
            name='exploration_planner_node',
            parameters=[config_file],
            output='screen'
        )
    motion_controller_node=Node(
            package='artibot_navigation',
            executable='motion_controller_node.py',
            name='motion_controller_node',
            parameters=[config_file],
            output='screen'
        )
    object_tracking_planner_node=Node(
            package='artibot_navigation',
            executable='object_tracking_planner_node.py',
            name='object_tracking_planner_node',
            parameters=[config_file],
            output='screen'
        )
    ld.add_action(config_file_arg)
    ld.add_action(collision_avoidance_planner_node)
    ld.add_action(exploration_planner_node)
    ld.add_action(motion_controller_node)
    ld.add_action(object_tracking_planner_node)

    return ld


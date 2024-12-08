#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('artibot_navigation'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Add rplidar_ros node
    rplidar_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rplidar_ros', 'rplidar_composition',
            '--ros-args',
            '-p', 'serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            '-p', 'frame_id:=laser_joint',
            '-p', 'angle_compensate:=true',
            '-p', 'scan_mode:=Standard',
            '-p', 'serial_baudrate:=115200'
        ],
        output='screen'
    )

    # Add rf2o_laser_odometry launch
    rf2o_odometry_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'rf2o_laser_odometry', 'rf2o_laser_odometry.launch.py'
        ],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher,
        rplidar_node,
        rf2o_odometry_launch
    ])

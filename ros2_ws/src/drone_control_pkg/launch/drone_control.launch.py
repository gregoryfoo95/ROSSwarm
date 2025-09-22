#!/usr/bin/env python3

"""
Launch file for C++ drone control system

This launch file starts:
- Simple drone monitor (C++)
- Test flight controller (C++)
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for C++ drone control system"""

    # Simple drone monitor node (C++)
    monitor_node = Node(
        package='drone_control_pkg',
        executable='simple_drone_monitor',
        name='drone_monitor',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Test flight node (C++) - can be used for interactive commands
    flight_node = Node(
        package='drone_control_pkg',
        executable='test_flight',
        name='flight_controller',
        output='screen',
        parameters=[{'interactive': False}],  # Set to automated mode by default
        respawn=False  # Don't respawn test flight node
    )

    # Launch description
    return LaunchDescription([
        LogInfo(msg='Starting C++ drone control system...'),

        monitor_node,
        # flight_node,  # Commented out - run separately when needed

        LogInfo(msg='C++ drone control system launched')
    ])
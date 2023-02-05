#!/usr/bin/env python
import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch file."""
    config_file = os.path.join(
        get_package_share_directory('ndt_locator'),
        'config/config.yaml'
    )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1', 'base_link', 'os_sensor'
        ]
    )

    map_publisher_node = launch_ros.actions.Node(
        package='ndt_locator',
        executable='map_publisher_node',
        parameters=[config_file],
        output='screen'
    )

    ndt_locator_node = launch_ros.actions.Node(
        package='ndt_locator',
        executable='ndt_node',
        parameters=[config_file],
        remappings=[('/scan', '/os_cloud_node/points'),],
        output='screen'
    )

    ld = launch.LaunchDescription([
        tf,
        map_publisher_node,
        ndt_locator_node,
    ])

    return ld

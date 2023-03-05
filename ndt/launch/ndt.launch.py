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

    rviz_file = os.path.join(
        get_package_share_directory('ndt_locator'),
        'rviz/config.rviz'
    )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1', 'base_link', 'os_sensor'
        ]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        name='rviz2',
        output='screen'
    )

    map_publisher_node = launch_ros.actions.Node(
        package='ndt_locator',
        executable='map_publisher_node',
        parameters=[config_file],
        output='screen'
    )

    lidar_processor_node = launch_ros.actions.Node(
        package='ndt_locator',
        executable='lidar_processor_node',
        parameters=[config_file],
        remappings=[
            ('/rawscan', '/os_cloud_node/points'),
        ],
        output='screen'
    )

    ndt_locator_node = launch_ros.actions.Node(
        package='ndt_locator',
        executable='ndt_node',
        parameters=[config_file],
        remappings=[
            ('/initial_pose', '/initialpose'),
        ],
        output='screen'
    )

    ld = launch.LaunchDescription([
        tf,
        map_publisher_node,
        lidar_processor_node,
        ndt_locator_node,
        rviz_node,
    ])

    return ld

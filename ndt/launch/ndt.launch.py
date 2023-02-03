#!/usr/bin/env python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ndt_locator',
            executable='ndt_node',
            output='screen',
            parameters=[{'map_file': 'maps/map.pcd'}]
        ),
    ])

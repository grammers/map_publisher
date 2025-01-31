import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    grid_map = os.path.join(
        get_package_share_directory('map_publisher'),
        'maps/'
        )
            
    node=Node(
        package = 'map_publisher',
        name = 'map_publisher',
        executable = 'map_publisher',
        parameters=[
            {'path':grid_map},
            {'map_name':'map'}
        ]
    )

    #parameters = [grid_map]

    ld.add_action(node)
    return ld

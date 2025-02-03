import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    map_file = LaunchConfiguration('map')

    '''
    grid_map = os.path.join(
        get_package_share_directory('map_publisher'),
        'maps/'
        )
    map_file = os.path.join(
        get_package_share_directory('map_publisher'),
        'maps',
        'map.yaml'
        )
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))
    '''

    map_file_arg = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(
            get_package_share_directory('map_publisher'),
            'maps',
            'map.yaml'))
            
    node=Node(
        package = 'map_publisher',
        name = 'map_publisher',
        executable = 'map_publisher',
        parameters=[
            {'map': map_file}
        ]
    )

    #parameters = [grid_map]

    ld.add_action(node)
    #return ld
    return LaunchDescription([
        map_file_arg,
        node
    ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
   
    pathserver_utm = Node(
        package='webots_ros2_pioneer3at',
        executable='pathserver_utm',
        name='pathserver_utm',
        output='screen',
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(pathserver_utm)

    return ld
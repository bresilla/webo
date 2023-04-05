import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    package_dir = get_package_share_directory('webots_ros2_pioneer3at')
    params_file = os.path.join(package_dir, 'params', 'navi.yaml')
    map_file = os.path.join(package_dir, 'maps', 'map.yaml')


    pathserver_utm = Node(
        package='webots_ros2_pioneer3at',
        executable='pathserver_utm',
        name='pathserver_utm',
        output='screen',
    )

    # Launch the ROS 2 Navigation Stack
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {'map': map_file,
                            'use_sim_time': use_sim_time,
                            'params_file': params_file}.items()
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(pathserver_utm)
    ld.add_action(start_ros2_navigation_cmd)

    return ld
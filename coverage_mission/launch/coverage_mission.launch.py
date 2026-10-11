import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('robot_bring_up'), 'config', 'coverage_mission.yaml')
    default_door_teb = os.path.join(
        get_package_share_directory('robot_bring_up'), 'config', 'teb_door.yaml')
    params_file = LaunchConfiguration('params_file')
    door_teb_profile_file = LaunchConfiguration('door_teb_profile_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('door_teb_profile_file', default_value=default_door_teb),
        Node(
            package='coverage_mission',
            executable='coverage_mission_node',
            name='coverage_mission_node',
            output='screen',
            parameters=[params_file, {
                'door_teb_profile_file': door_teb_profile_file,
            }],
        ),
    ])

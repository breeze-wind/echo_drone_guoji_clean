import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_default = os.path.join(
        get_package_share_directory('target_perception'),
        'config', 'target_perception.yaml')
    config_file = LaunchConfiguration('config_file')
    show_window = LaunchConfiguration('show_window')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_default),
        DeclareLaunchArgument(
            'show_window', default_value='true',
            description='Show OpenCV detection preview window'),
        Node(
            package='target_perception',
            executable='target_camera_node',
            name='target_camera',
            output='screen',
            parameters=[config_file]),
        Node(
            package='target_perception',
            executable='target_localization_node',
            name='target_localization',
            output='screen',
            parameters=[config_file, {
                'show_window': ParameterValue(show_window, value_type=bool),
            }]),
        Node(
            package='target_perception',
            executable='map_target_localizer_node',
            name='map_target_localizer',
            output='screen',
            parameters=[config_file]),
    ])

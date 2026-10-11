import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('electromagnet_node'), 'config', 'electromagnet.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(
            package='electromagnet_node',
            executable='electromagnet_node',
            name='electromagnet_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'port': LaunchConfiguration('port'),
                'dry_run': LaunchConfiguration('dry_run'),
            }],
        ),
    ])

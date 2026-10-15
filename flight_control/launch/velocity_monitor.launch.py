"""单独启动只读的 Nav2/MAVROS/Point-LIO 速度比较节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('flight_control'), 'config',
                'velocity_monitor.yaml',
            ]),
            description='只读速度比较节点的参数文件。'),
        Node(
            package='flight_control',
            executable='velocity_monitor_node',
            name='velocity_monitor',
            output='screen',
            parameters=[config_file],
        ),
    ])

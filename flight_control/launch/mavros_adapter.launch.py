"""只启动 MAVROS 兼容适配器。

当 MAVROS 已经单独运行，或者只想做 ROS 内部 dry-run 时使用这个入口。
默认 YAML 保持 `dry_run: true`，除非调用方覆盖参数，否则服务调用只记录
日志，不真正发送。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 适配器配置允许从命令行替换，台架测试时不用改文件就能切换 dry-run
    # 和真实飞控设置。
    config_file = LaunchConfiguration('config_file')

    adapter_node = Node(
        package='flight_control',
        executable='mavros_adapter_node',
        name='mavros_adapter',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('flight_control'),
                'config',
                'mavros_adapter.yaml',
            ]),
            description='MAVROS adapter 参数文件。',
        ),
        adapter_node,
    ])

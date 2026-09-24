# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    params_file = LaunchConfiguration('params_file')
    dry_run = LaunchConfiguration('dry_run')

    node_01 = Node(
        package="servo_node",
        executable="servo_node",
        output="screen",
        name="servo_node",
        parameters=[params_file, {'dry_run': dry_run}],
        respawn=True # 重启
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution([
                    FindPackageShare('robot_bring_up'),
                    'config',
                    'hardware',
                    'servo.yaml',
                ]),
            ),
            DeclareLaunchArgument('dry_run', default_value='true'),
            node_01,
        ]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description

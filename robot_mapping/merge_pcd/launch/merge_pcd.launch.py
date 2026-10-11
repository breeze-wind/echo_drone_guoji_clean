# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    default_yaml_path = os.path.join(
        get_package_share_directory('robot_bring_up'),
        'config',
        'drone.yaml'
    )

    yaml_path = LaunchConfiguration(
        'params_file',
        default=default_yaml_path
    )
    paths_path = os.path.join(
        get_package_share_directory('robot_bring_up'), 'config', 'map_paths.yaml')
    with open(paths_path, 'r') as paths_stream:
        map_paths = yaml.safe_load(paths_stream)['map_paths']
    path_parameters = {
        'pcd_folder': map_paths.get('merged_pcd_dir', '')
    }
    declare_yaml_path = DeclareLaunchArgument(
        'params_file',
        default_value=yaml_path,
        description='Full path to the pcd2pgm configuration file to load'
    )

    merge_pcd_node = Node(
        package="merge_pcd",
        executable="merge_pcd_node",
        output="screen",
        parameters=[yaml_path, path_parameters],
        name="merge_pcd_node"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [declare_yaml_path, merge_pcd_node]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description

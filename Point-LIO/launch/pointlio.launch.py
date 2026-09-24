import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    package_path = get_package_share_directory('point_lio')
    config_package_path = get_package_share_directory('robot_bring_up')
    default_config_path = os.path.join(config_package_path, 'config', 'drone.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz_cfg', 'loam_livox.rviz')

    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        parameters=[config_path],
        output='screen',
        respawn=True,
        respawn_delay=1,
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(point_lio_node)
    # ld.add_action(rviz_node)

    return ld

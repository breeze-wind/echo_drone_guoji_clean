import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bring_up')
    livox_share = get_package_share_directory('livox_ros_driver2')
    default_params = os.path.join(bringup_share, 'config', 'drone.yaml')
    frames_path = os.path.join(bringup_share, 'config', 'static_frames.yaml')
    with open(frames_path, 'r') as frames_file:
        static_frames = yaml.safe_load(frames_file)['static_frames']

    params_file = LaunchConfiguration('params_file')
    save_interval = LaunchConfiguration('pcd_save_interval')

    def static_tf(name):
        frame = static_frames[name]
        values = frame['translation'] + frame['rotation']
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name + '_broadcaster',
            arguments=[str(value) for value in values] + [frame['parent'], frame['child']],
            output='screen',
        )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'msg_MID360s_launch.py')))

    point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laser_mapping',
        output='screen',
        respawn=False,
        parameters=[
            params_file,
            {
                'pcd_save.pcd_save_en': True,
                'pcd_save.interval': ParameterValue(save_interval, value_type=int),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Point-LIO and sensor parameter file'),
        DeclareLaunchArgument(
            'pcd_save_interval', default_value='900',
            description='Number of accumulated scans per saved PCD chunk'),
        LogInfo(msg=['Requesting Point-LIO PCD capture; chunk interval=', save_interval]),
        static_tf('map_to_odom'),
        static_tf('livox_raw_to_livox'),
        static_tf('livox_to_mavlink_body'),
        static_tf('livox_to_camera'),
        livox_driver,
        TimerAction(period=8.0, actions=[point_lio]),
    ])

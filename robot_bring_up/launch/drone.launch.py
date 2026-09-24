# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import TimerAction

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""

    if_rviz = True
    if_sim = False
    if_map = False   # 只启动robot_description、雷达驱动、point-lio #map1前True map3 False

    point_lio_path = get_package_share_directory("point_lio")
    robot_bringup_path = get_package_share_directory("robot_bring_up")
    obstacle_segmentation_path = get_package_share_directory("obstacle_segmentation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup") #nav2_bringup功能包
    livox_driver_path = get_package_share_directory("livox_ros_driver2")

    yaml_path = os.path.join(robot_bringup_path, "config", "drone.yaml")

    param_if_map = LaunchConfiguration("if_map", default=if_map)
    declare_if_map = DeclareLaunchArgument(
        "if_map",
        default_value=param_if_map,
        description="Whether to run map",
    )
    param_yaml_path = LaunchConfiguration("params_file", default=yaml_path)
    declare_yaml_path = DeclareLaunchArgument(
        "params_file",
        default_value=param_yaml_path,
        description="Full path to the configuration file to load",
    )
    param_launch_rviz = LaunchConfiguration("launch_rviz", default=if_rviz)
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value=param_launch_rviz,
        description="Whether to run rviz",
    )
    param_rviz_config_dir = LaunchConfiguration(
        "rviz_config_dir",
        default=os.path.join(robot_bringup_path,"config","betterRvizConfig.rviz"), #(nav2_bringup_dir, "rviz", "nav2_default_view.rviz"),
    )
    declare_rviz_config_dir = DeclareLaunchArgument(
        "rviz_config_dir",
        default_value=param_rviz_config_dir,
        description="Full path to the rviz config file to load",
    )
    param_launch_gazebo = LaunchConfiguration("launch_gazebo", default=if_sim)
    declare_launch_gazebo = DeclareLaunchArgument(
        "launch_gazebo",
        default_value=param_launch_gazebo,
        description="Whether to run gazebo",
    )
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [livox_driver_path, "/launch_ROS2", "/msg_MID360s_launch.py"]
        ),
    )
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [point_lio_path, "/launch", "/pointlio.launch.py"]
        ),
    )
    obstacle_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [obstacle_segmentation_path, "/launch", "/obstacle_segmentation.launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
        }.items(),
        condition=UnlessCondition(param_if_map),
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_bringup_path, "/launch", "/bringup_launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
            "use_sim_time": param_launch_gazebo,
        }.items(),
        condition=UnlessCondition(param_if_map),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", param_rviz_config_dir],
        parameters=[{"use_sim_time": param_launch_gazebo}],
        output="screen",
        condition=IfCondition(param_launch_rviz),
    )
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0.0', '0.0', '0.39',  '0.0', '0.0', '0.0', '1.0','map', 'odom']
    )
    # livox_raw -> livox: x'=-y, y'=x, z'=z (+90deg rotation about z)
    livox_raw_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_raw_to_livox_broadcaster',
        arguments=['0.0', '0.0', '0.0',  '0.0', '0.0', '0.7071068', '0.7071068','livox_raw', 'livox']
    )
    livox_to_mavlink_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_mavlink_body_broadcaster',
        arguments=['0.0', '0.0', '-0.08',  '1.0', '0.0', '0.0', '0.0','livox', 'mavlink_body']
    )
    livox_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_camera_broadcaster',
        arguments=['0.065', '-0.065', '-0.26',  '0.3827', '0.9239', '0.0', '0.0','livox', 'camera_link']
    )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    # 我尝试下来，在lio启动时，系统不能负载太高，因此，选择在lio启动后再启动其他节点，这个时间可以根据实际情况调整
    list = [
        map_to_odom,
        livox_raw_to_livox,
        livox_driver_launch,
        declare_launch_gazebo,
        declare_yaml_path,
        declare_launch_rviz,
        declare_if_map,
        declare_rviz_config_dir,
        livox_to_mavlink_body,
        livox_to_camera,
        TimerAction(
            period=8.0,
            actions=[
                point_lio_launch
            ],
        ),
        navigation_launch,
        rviz_node,
        obstacle_segmentation_launch
    ]

    # 返回让ROS2根据launch描述执行节点
    return LaunchDescription(list)

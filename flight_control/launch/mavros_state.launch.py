"""启动最小 MAVROS 节点，用于检查飞控状态和话题。

当前工作区的 Foxy 环境不能稳定解析上游 MAVROS XML launch 文件，因为其中
仍有旧 launch 替换语法。这个 Python 包装只传入 PX4 心跳检查所需的串口
URL、目标 ID 和协议参数，直接启动 `mavros_node`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 暂时不传入大插件配置；在目标 Foxy/ARM 镜像上确认插件过滤方案前，
    # 只读状态检查保持简单。
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        respawn=False,
        parameters=[
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'target_system_id': ParameterValue(
                    LaunchConfiguration('tgt_system'),
                    value_type=int,
                ),
                'target_component_id': ParameterValue(
                    LaunchConfiguration('tgt_component'),
                    value_type=int,
                ),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            },
        ],
    )

    return LaunchDescription([
        # 默认支持 udev 管理的飞控链接；WSL/USBIP 下 udev 可能未生效，
        # 所以也允许直接使用 /dev/ttyACM0。
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:230400',
            description='MAVROS 连接飞控的串口 URL。',
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='MAVROS 目标系统 ID。',
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='MAVROS 目标组件 ID。',
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='MAVROS 使用的 MAVLink 协议版本。',
        ),
        mavros_node,
    ])

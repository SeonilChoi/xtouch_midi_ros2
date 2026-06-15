import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    motor_manager_share = get_package_share_directory('ros2_motor_manager')
    default_config = os.path.join(
        motor_manager_share,
        'config',
        'example_canopen_zeroerr.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Absolute path to ros2_motor_manager YAML config.',
    )
    command_topic_arg = DeclareLaunchArgument(
        'command_topic',
        default_value='motor_command',
        description='MotorStatus command topic consumed by ros2_motor_manager.',
    )
    publish_raw_topics_arg = DeclareLaunchArgument(
        'publish_raw_topics',
        default_value='true',
        description='Also publish /xtouch/fader/chN and /xtouch/touch/chN topics.',
    )

    return LaunchDescription([
        config_file_arg,
        command_topic_arg,
        publish_raw_topics_arg,
        Node(
            package='xtouch_midi',
            executable='xtouch_node',
            name='xtouch_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'command_topic': LaunchConfiguration('command_topic'),
                'publish_raw_topics': ParameterValue(
                    LaunchConfiguration('publish_raw_topics'),
                    value_type=bool,
                ),
            }],
        ),
    ])

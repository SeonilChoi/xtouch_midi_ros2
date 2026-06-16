from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    midi_topic_arg = DeclareLaunchArgument(
        'midi_topic',
        default_value='/xtouch/midi',
        description='Full X-Touch MIDI state topic.',
    )
    publish_period_ms_arg = DeclareLaunchArgument(
        'publish_period_ms',
        default_value='5',
        description='Periodic /xtouch/midi publish period in milliseconds.',
    )
    encoder_relative_mode_arg = DeclareLaunchArgument(
        'encoder_relative_mode',
        default_value='true',
        description='Treat rotary encoder CC values as relative MCU deltas.',
    )
    button_led_feedback_arg = DeclareLaunchArgument(
        'button_led_feedback',
        default_value='true',
        description='Send MIDI note feedback to keep button LEDs in sync.',
    )
    initial_dial_value_arg = DeclareLaunchArgument(
        'initial_dial_value',
        default_value='0',
        description='Initial accumulated dial value, clamped to 0..127.',
    )

    return LaunchDescription([
        midi_topic_arg,
        publish_period_ms_arg,
        encoder_relative_mode_arg,
        button_led_feedback_arg,
        initial_dial_value_arg,
        Node(
            package='xtouch_midi',
            executable='xtouch_node',
            name='xtouch_node',
            output='screen',
            parameters=[{
                'midi_topic': LaunchConfiguration('midi_topic'),
                'publish_period_ms': ParameterValue(
                    LaunchConfiguration('publish_period_ms'),
                    value_type=int,
                ),
                'encoder_relative_mode': ParameterValue(
                    LaunchConfiguration('encoder_relative_mode'),
                    value_type=bool,
                ),
                'button_led_feedback': ParameterValue(
                    LaunchConfiguration('button_led_feedback'),
                    value_type=bool,
                ),
                'initial_dial_value': ParameterValue(
                    LaunchConfiguration('initial_dial_value'),
                    value_type=int,
                ),
            }],
        ),
    ])

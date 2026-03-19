#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_frame_id', default_value='respeaker_base',
                              description='Frame ID for the microphone'),
        DeclareLaunchArgument('doa_xy_offset', default_value='0.0'),
        DeclareLaunchArgument('doa_yaw_offset', default_value='90.0'),
        DeclareLaunchArgument('update_rate', default_value='10.0'),
        DeclareLaunchArgument('main_channel', default_value='0'),
        DeclareLaunchArgument('suppress_pyaudio_error', default_value='true'),
        DeclareLaunchArgument('save_audio', default_value='true'),
        DeclareLaunchArgument('audio_output_dir', default_value='/tmp/respeaker_audio'),

        Node(
            package='respeaker_ros',
            executable='respeaker_node',
            name='respeaker_node',
            output='screen',
            parameters=[{
                'sensor_frame_id': LaunchConfiguration('sensor_frame_id'),
                'doa_xy_offset': LaunchConfiguration('doa_xy_offset'),
                'doa_yaw_offset': LaunchConfiguration('doa_yaw_offset'),
                'update_rate': LaunchConfiguration('update_rate'),
                'main_channel': LaunchConfiguration('main_channel'),
                'suppress_pyaudio_error': LaunchConfiguration('suppress_pyaudio_error'),
                'save_audio': LaunchConfiguration('save_audio'),
                'audio_output_dir': LaunchConfiguration('audio_output_dir'),
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transformer',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'respeaker_base'],
        ),
        Node(
            package='respeaker_ros',
            executable='speech_to_text',
            name='speech_to_text',
            parameters=[{
                'language': 'ko-KR',
                'fallback_languages': ['en-US'],
                'self_cancellation': True,
                'tts_tolerance': 0.5,
                'save_audio': LaunchConfiguration('save_audio'),
                'audio_output_dir': LaunchConfiguration('audio_output_dir'),
            }],
            remappings=[('audio', 'speech_audio')],
        ),
    ])

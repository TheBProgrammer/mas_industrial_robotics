"""Launch file for the barrier tape mock perception node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the barrier tape mock node."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            choices=['true', 'false'],
            description='Use simulation clock (must match nav2 use_sim_time)'),
        DeclareLaunchArgument(
            'tape_start_x', default_value='4.0',
            description='Tape start X in map frame (m)'),
        DeclareLaunchArgument(
            'tape_start_y', default_value='1.5',
            description='Tape start Y in map frame (m)'),
        DeclareLaunchArgument(
            'tape_end_x', default_value='4.0',
            description='Tape end X in map frame (m)'),
        DeclareLaunchArgument(
            'tape_end_y', default_value='3.0',
            description='Tape end Y in map frame (m)'),
        DeclareLaunchArgument(
            'num_points', default_value='40',
            description='Number of points along the tape'),
        DeclareLaunchArgument(
            'publish_rate', default_value='10.0',
            description='Publish rate in Hz'),

        Node(
            package='skratch_navigation',
            executable='barrier_tape_mock',
            name='barrier_tape_mock',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'tape_start_x': LaunchConfiguration('tape_start_x'),
                'tape_start_y': LaunchConfiguration('tape_start_y'),
                'tape_end_x': LaunchConfiguration('tape_end_x'),
                'tape_end_y': LaunchConfiguration('tape_end_y'),
                'num_points': LaunchConfiguration('num_points'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': 'map',
            }],
        ),
    ])

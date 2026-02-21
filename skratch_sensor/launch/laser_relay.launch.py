from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Relay /front_scan → /scan_combined
        Node(
            package='topic_tools',
            executable='relay',
            name='front_laser_scanner_repub',
            output='screen',
            arguments=['/front_scan', '/scan_combined']
        ),

        # Relay /rear_scan → /scan_combined
        Node(
            package='topic_tools',
            executable='relay',
            name='rear_laser_scanner_repub',
            output='screen',
            arguments=['/rear_scan', '/scan_combined']
        ),

    ])

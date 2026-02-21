import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('skratch_sensor'), 'config')
    params_file = os.path.join(config_dir, 'dual_laser_merger_params.yaml')

    return LaunchDescription([
        Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_alaser_merger',
            output='screen',
            parameters=[params_file]
        ),
    ])

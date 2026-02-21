from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('map_name', default_value='rc_arena_sim',
                          description='Map name/subdirectory (e.g., rc_arena_sim, eval_arena)')
]


def generate_launch_description():
    pkg_skratch_localization = get_package_share_directory('skratch_localization')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_skratch_navigation = get_package_share_directory('skratch_mapping')

    map_name = LaunchConfiguration('map_name')

    localization_params_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [pkg_skratch_localization, 'config', 'localization.yaml']),
        description='Localization parameters')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_skratch_navigation, 'maps', map_name, 'map.yaml']),
        description='Full path to map yaml file to load')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
            launch_arguments={'namespace': namespace,
                              'map': LaunchConfiguration('map'),
                              'use_sim_time': use_sim_time,
                              'params_file': LaunchConfiguration('params')}.items()),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localization_params_arg)
    ld.add_action(map_arg)
    ld.add_action(localization)
    return ld

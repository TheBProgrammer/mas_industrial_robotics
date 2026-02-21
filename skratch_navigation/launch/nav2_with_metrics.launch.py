from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([
                              get_package_share_directory('skratch_navigation'),
                              'config', 'nav2_params_dwb_theta.yaml']),
                          description='Nav2 parameters'),
    DeclareLaunchArgument('goals_config',
                          default_value=PathJoinSubstitution([
                              get_package_share_directory('skratch_navigation'),
                              'config', 'goals_config.yaml']),
                          description='Goals configuration file'),
    DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
]


def launch_setup(context, *args, **kwargs):
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = LaunchConfiguration('params_file')
    goals_config = LaunchConfiguration('goals_config')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                  ('use_sim_time', use_sim_time),
                  ('params_file', nav2_params.perform(context)),
                  ('use_composition', 'False'),
                  ('namespace', namespace_str)
                ]
        ),
    ])

    # Nav metrics logger with sequential navigation
    nav_metrics_logger = Node(
        package='skratch_navigation',
        executable='nav_metrics_logger',
        name='nav_metrics_logger',
        output='screen',
        parameters=[
            goals_config.perform(context),
            {'use_sim_time': use_sim_time}
        ]
    )

    return [nav2, nav_metrics_logger]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

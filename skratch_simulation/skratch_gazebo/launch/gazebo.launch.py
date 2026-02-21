#!/usr/bin/env python3
# Authors: Anudeep Sajja

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # lc = LaunchContext()
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    # declare_use_sim_argument = DeclareLaunchArgument(
    #     'use_sim',
    #     default_value='true',
    #     description='Whether to include Gazebo simulation plugins'
    # )

    # use_sim = LaunchConfiguration('use_sim')

    x_pose = LaunchConfiguration('x_pose', default='1.43')
    y_pose = LaunchConfiguration('y_pose', default='0.24')
    yaw = LaunchConfiguration('yaw_pose', default='1.57')

    
    world = os.path.join( 
        get_package_share_directory('skratch_gazebo'), 
        'worlds', 
        # 'eval_arena.world',
        'atwork_world.sdf' 
        )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("skratch_description"),
                    "urdf",
                    "skratch_urdf.xacro" 
                ]
            )
        ]
    )

    robot_state_pub_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_content, value_type=str)
        }],
    )

    joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    spawn_robot_gazebo_cmd = Node(package='gazebo_ros',
                                  executable='spawn_entity.py',
                                  arguments=['-topic', 'robot_description',
                                             '-entity', 'robile',
                                             '-x', x_pose,
                                             '-y', y_pose,
                                            #  '-Y', yaw,
                                             ],
                                  output='screen')

    # RViz configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('skratch_gazebo'),
        'config',
        'rviz',
        'skratch_visualization.rviz'
    )

    rviz_cmd = Node(package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',
                    )

    static_transform_cmd = Node(package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                arguments=["0", "0", "0", "0", "0",
                                           "0", "base_footprint", "base_link"]
                                )
    
    lidar_merge_relay_1 = Node(package='topic_tools',
                            executable='relay',
                            name='front_laser_scanner_repub',
                            output='screen',
                            arguments=['/front_scan', '/scan_combined']
                            )
    
    lidar_merge_relay_2 = Node(package='topic_tools',
                                executable='relay',
                                name='rear_laser_scanner_repub',
                                output='screen',
                                arguments=['/rear_scan', '/scan_combined']
                                )

    dual_laser_merger_config_dir = os.path.join(get_package_share_directory('skratch_sensor'), 'config')
    dual_laser_merger_params_file = os.path.join(dual_laser_merger_config_dir, 'dual_laser_merger_params.yaml')

    dual_laser_merger_node = Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
            parameters=[dual_laser_merger_params_file]
        )

    nodes = [
        rviz_cmd,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_pub_cmd,
        spawn_robot_gazebo_cmd,
        # static_transform_cmd,
        joint_state_publisher_cmd,
        # lidar_merge_relay_1,
        # lidar_merge_relay_2,
        dual_laser_merger_node
    ]

    return LaunchDescription(nodes)


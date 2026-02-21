import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    # Package paths
    pkg_path = FindPackageShare('skratch_description').find('skratch_description')

    # Xacro processing
    xacro_file = os.path.join(pkg_path, 'urdf', 'skratch_urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # RViz config using PathJoinSubstitution
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('skratch_description'), 'config', 'skratch_description.rviz']
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_node_gui = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Optional debug message to confirm RViz path
    log_rviz_path = LogInfo(
        msg=['RViz config file path: ', rviz_config_file]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        log_rviz_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        joint_state_publisher_node_gui
    ])

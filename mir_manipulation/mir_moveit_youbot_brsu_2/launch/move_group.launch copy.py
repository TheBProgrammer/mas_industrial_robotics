from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    # load robot configuration
    moveit_config = (
        MoveItConfigsBuilder("youbot", package_name="mir_moveit_youbot_brsu_2")
        .robot_description("config/youbot.urdf.xacro")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .robot_description_semantic(file_path="config/youbot.srdf")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            run_move_group_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
        ]
    )

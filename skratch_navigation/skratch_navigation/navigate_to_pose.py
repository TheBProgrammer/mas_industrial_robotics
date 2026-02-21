#!/usr/bin/env python3
"""
Navigate robot to named poses stored in YAML file.

Usage: ros2 run skratch_navigation navigate <pose_name>
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import math


from ament_index_python.packages import get_package_share_directory


class NavigateToPoseNode(Node):
    def __init__(self):
        super().__init__('navigate_to_pose')

        # Declare parameters
        self.declare_parameter('frame_id', 'map')
        self.frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose,
                                       'navigate_to_pose')

        # Package setup
        self.package_name = 'skratch_navigation'
        self.config_file = 'navigation_goals.yaml'

        self.get_logger().info('Navigate to Pose Node initialized')
        self.get_logger().info(f'Using goals frame: {self.frame_id}')

    def get_config_file_path(self):
        """Get the full path to the navigation goals YAML file."""
        try:
            package_share_directory = get_package_share_directory(self.package_name)
            config_path = os.path.join(
                package_share_directory, 'config', self.config_file
            )

            if not os.path.exists(config_path):
                self.get_logger().error(
                    f'Configuration file not found: {config_path}'
                )
                self.get_logger().error(
                    'Please run save_poses first to create navigation goals.'
                )
                return None

            return config_path

        except Exception as e:
            self.get_logger().error(f'Error determining config path: {e}')
            return None

    def load_pose_from_file(self, pose_name):
        """Load pose data from YAML file."""
        config_path = self.get_config_file_path()
        if config_path is None:
            return None

        try:
            with open(config_path, 'r') as f:
                poses = yaml.safe_load(f)

            if poses is None or pose_name not in poses:
                self.get_logger().error(
                    f'Pose "{pose_name}" not found in {self.config_file}'
                )
                self.get_logger().info(
                    f'Available poses: '
                    f'{list(poses.keys()) if poses else "None"}'
                )
                return None

            pose_data = poses[pose_name]
            self.get_logger().info(
                f'Loaded pose "{pose_name}": {pose_data}'
            )
            return pose_data

        except Exception as e:
            self.get_logger().error(f'Failed to load pose from file: {e}')
            return None

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        return [0.0, 0.0, qz, qw]  # [x, y, z, w]

    def create_pose_stamped(self, pose_data):
        """Create PoseStamped message from pose data [x, y, yaw]."""
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        pose.pose.position.x = float(pose_data[0])
        pose.pose.position.y = float(pose_data[1])
        pose.pose.position.z = 0.0

        # Set orientation from yaw
        quat = self.yaw_to_quaternion(float(pose_data[2]))
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    def navigate_to_pose(self, pose_name):
        """Navigate to named pose using Nav2."""
        # Load pose from file
        pose_data = self.load_pose_from_file(pose_name)
        if pose_data is None:
            return False

        # Store goal pose for feedback callback
        self.current_goal_pose = pose_data
        self.current_goal_name = pose_name

        # Wait for action server
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            self.get_logger().error('Make sure Nav2 is running.')
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(pose_data)

        self.get_logger().info(
            f'Sending navigation goal to pose "{pose_name}"...'
        )
        self.get_logger().info(
            f'  Position: x={pose_data[0]:.2f}, y={pose_data[1]:.2f}'
        )
        self.get_logger().info(
            f'  Orientation: yaw={pose_data[2]:.2f} rad '
            f'({math.degrees(pose_data[2]):.1f}°)'
        )

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            return False

        self.get_logger().info(
            'Navigation goal accepted. Robot is moving...'
        )

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Successfully navigated to pose "{pose_name}"!'
            )
            return True
        else:
            self.get_logger().error(
                f'Navigation failed with status: {result.status}'
            )
            return False

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle in radians."""
        # Extract quaternion components
        x, y, z, w = quat.x, quat.y, quat.z, quat.w

        # Calculate yaw (rotation around z-axis)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def feedback_callback(self, feedback_msg):
        """Process navigation feedback."""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose

        goal_pose = self.current_goal_pose

        theta = self.quaternion_to_yaw(current_pose.orientation)
        goal_x, goal_y, goal_yaw = goal_pose[0], goal_pose[1], goal_pose[2]

        self.get_logger().info(
            f'Navigation progress - '
            f'Current: x={current_pose.position.x:.2f}, '
            f'y={current_pose.position.y:.2f}, '
            f'theta={theta:.2f} rad ({math.degrees(theta):.1f}°) | '
            f'Goal: x={goal_x:.2f}, y={goal_y:.2f}, '
            f'theta={goal_yaw:.2f} rad ({math.degrees(goal_yaw):.1f}°)',
            throttle_duration_sec=2.0  # Log every 2 seconds
        )


def main(args=None):
    rclpy.init(args=args)

    # Check for pose name argument
    if len(sys.argv) < 2:
        print('\n' + '='*60)
        print('ERROR: No pose name provided!')
        print('='*60)
        print('Usage: ros2 run skratch_navigation navigate <pose_name>')
        print('Example: ros2 run skratch_navigation navigate home')
        print('='*60 + '\n')
        rclpy.shutdown()
        sys.exit(1)

    pose_name = sys.argv[1]

    node = NavigateToPoseNode()

    try:
        success = node.navigate_to_pose(pose_name)
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation cancelled by user')
        sys.exit(1)
    except Exception as e:
        node.get_logger().error(f'Fatal error: {e}')
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

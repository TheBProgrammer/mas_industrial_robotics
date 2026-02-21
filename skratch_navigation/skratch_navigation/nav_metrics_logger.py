#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import csv
import os
import psutil
from datetime import datetime
import numpy as np
from tf_transformations import quaternion_from_euler


class NavMetricsLogger(Node):
    def __init__(self):
        super().__init__('nav_metrics_logger')

        # Parameters
        # CSV will be created in source package eval directory
        # Get workspace root and construct path to source eval folder
        try:
            from ament_index_python.packages import (
                get_package_share_directory
            )
            pkg_share_dir = get_package_share_directory(
                'skratch_navigation'
            )
            # Navigate from install/share back to src
            workspace_root = os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    pkg_share_dir
                )))
            )
            src_eval_dir = os.path.join(
                workspace_root, 'src', 'skratch_navigation', 'eval'
            )
            default_csv_path = os.path.join(
                src_eval_dir, 'nav_metrics_sequential.csv'
            )
        except Exception:
            # Fallback: use relative path from this file
            src_pkg_dir = os.path.dirname(os.path.dirname(__file__))
            default_csv_path = os.path.join(
                src_pkg_dir, 'eval', 'nav_metrics_sequential.csv'
            )

        self.declare_parameter('csv_path', default_csv_path)
        self.declare_parameter('goal_poses', '')
        self.declare_parameter('delay_between_goals', 2.0)
        self.declare_parameter('controller_name', 'MPPI')
        self.declare_parameter('planner_name', 'SmacHybrid')

        self.csv_file_path = self.get_parameter('csv_path').value
        self.goal_poses_config = self.get_parameter('goal_poses').value
        self.delay_between_goals = (
            self.get_parameter('delay_between_goals').value
        )
        self.controller_name = self.get_parameter('controller_name').value
        self.planner_name = self.get_parameter('planner_name').value

        # Parse goal poses
        self.goal_poses = self.parse_goal_poses(self.goal_poses_config)
        self.current_goal_index = 0

        # Initialize CSV
        self.init_csv()

        # State Variables
        self.reset_metrics()

        # Action Client
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

        # Subscriptions for metrics
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(
            LaserScan, '/scan_combined', self.scan_callback, 10
        )

        # Timer for sequential navigation
        self.navigation_timer = None

        self.get_logger().info(
            f"Nav Metrics Logger Started. Logging to: "
            f"{self.csv_file_path}"
        )
        self.get_logger().info(f"Loaded {len(self.goal_poses)} goal poses")

        # Wait for action server and start navigation
        self.navigation_timer = self.create_timer(
            1.0, self.check_action_server_and_start
        )

    def parse_goal_poses(self, config):
        """Parse goal poses from parameter configuration."""
        import json

        goals = []
        if not config:
            self.get_logger().warn("No goal_poses configured!")
            return goals

        # Handle JSON string format
        if isinstance(config, str):
            try:
                config = json.loads(config)
            except json.JSONDecodeError as e:
                self.get_logger().error(
                    f"Failed to parse goal_poses JSON: {e}"
                )
                return goals

        for i, goal_dict in enumerate(config):
            try:
                x = float(goal_dict['x'])
                y = float(goal_dict['y'])
                yaw = float(goal_dict['yaw'])
                frame_id = goal_dict.get('frame_id', 'map')
                goals.append({
                    'x': x, 'y': y, 'yaw': yaw, 'frame_id': frame_id
                })
                self.get_logger().info(
                    f"Goal {i}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, "
                    f"frame={frame_id}"
                )
            except (KeyError, ValueError) as e:
                self.get_logger().error(f"Failed to parse goal {i}: {e}")

        return goals

    def check_action_server_and_start(self):
        """Check if action server is available and start navigation."""
        if self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info(
                "Action server available. Starting sequential navigation..."
            )
            # Destroy the repeating check timer
            if self.navigation_timer:
                self.destroy_timer(self.navigation_timer)
                self.navigation_timer = None
            # Start navigation after a short delay
            self._start_timer = self.create_timer(
                2.0, self._start_timer_callback
            )
        else:
            self.get_logger().info(
                "Waiting for navigate_to_pose action server..."
            )

    def _start_timer_callback(self):
        """One-shot callback to start first goal."""
        self.destroy_timer(self._start_timer)
        self.start_next_goal()

    def init_csv(self):
        headers = [
            'Controller', 'Planner', 'Goal_Index', 'Goal_X', 'Goal_Y',
            'Goal_Yaw', 'Timestamp', 'Outcome', 'Total_Time(s)',
            'Planning_Time(s)', 'Path_Length(m)', 'Path_Smoothness(rad)',
            'Avg_Linear_Vel(m/s)', 'Max_Linear_Vel(m/s)',
            'Avg_Angular_Vel(rad/s)', 'Max_Angular_Vel(rad/s)',
            'Avg_Lateral_Vel(m/s)', 'Max_Lateral_Vel(m/s)',
            'Max_Accel_Linear(m/s2)', 'Max_Jerk_Linear(m/s3)',
            'Min_Obstacle_Dist(m)', 'Near_Collisions_Count',
            'Avg_CPU_Usage(%)', 'Max_RAM_Usage(MB)',
            'Recovery_Count'
        ]

        # Ensure parent directory exists
        csv_dir = os.path.dirname(self.csv_file_path)
        if csv_dir and not os.path.exists(csv_dir):
            os.makedirs(csv_dir, exist_ok=True)
            self.get_logger().info(f"Created directory: {csv_dir}")

        # Write header if file doesn't exist or is empty
        file_is_new = (
            not os.path.exists(self.csv_file_path) or
            os.path.getsize(self.csv_file_path) == 0
        )
        if file_is_new:
            with open(self.csv_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                self.get_logger().info(
                    f"Created new CSV with headers: {self.csv_file_path}"
                )
        else:
            self.get_logger().info(
                f"Appending to existing CSV: {self.csv_file_path}"
            )

    def reset_metrics(self):
        self.is_navigating = False
        self.start_time = None
        self.end_time = None
        self.planning_start_time = None
        self.planning_end_time = None

        # Path
        self.distance_traveled = 0.0
        self.path_smoothness = 0.0
        self.last_odom_pos = None
        self.last_path_pose = None

        # Dynamics
        self.velocities_linear = []
        self.velocities_angular = []
        self.velocities_lateral = []
        self.accels_linear = []
        self.jerks_linear = []
        self.last_vel_linear = 0.0
        self.last_accel_linear = 0.0
        self.last_odom_time = None

        # Safety
        self.min_obstacle_dist = 999.0
        self.near_collision_count = 0

        # System
        self.cpu_usages = []
        self.ram_usages = []
        self.recovery_count = 0

    def start_next_goal(self):
        """Send the next goal in the sequence."""
        if self.current_goal_index >= len(self.goal_poses):
            self.get_logger().info(
                "All goals completed! Sequential navigation finished."
            )
            return

        goal_config = self.goal_poses[self.current_goal_index]
        self.get_logger().info(
            f"Starting navigation to goal "
            f"{self.current_goal_index + 1}/{len(self.goal_poses)}: "
            f"x={goal_config['x']:.2f}, y={goal_config['y']:.2f}, "
            f"yaw={goal_config['yaw']:.2f}"
        )

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = goal_config['frame_id']
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_config['x']
        goal_msg.pose.pose.position.y = goal_config['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = quaternion_from_euler(0, 0, goal_config['yaw'])
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # Reset metrics and start tracking
        self.reset_metrics()
        self.is_navigating = True
        self.start_time = self.get_clock().now()
        self.planning_start_time = self.start_time

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Goal {self.current_goal_index} was rejected!"
            )
            self.finalize_log('REJECTED')
            # Move to next goal after delay
            self._next_goal_timer = self.create_timer(
                self.delay_between_goals, self._next_goal_timer_callback
            )
            return

        self.get_logger().info(
            f"Goal {self.current_goal_index} accepted"
        )
        self._goal_handle = goal_handle

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        # Could log distance remaining, etc.
        pass

    def get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Goal {self.current_goal_index} succeeded!"
            )
            self.finalize_log('SUCCESS')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f"Goal {self.current_goal_index} was aborted"
            )
            self.finalize_log('ABORTED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f"Goal {self.current_goal_index} was canceled"
            )
            self.finalize_log('CANCELED')
        else:
            self.get_logger().warn(
                f"Goal {self.current_goal_index} ended with status: "
                f"{status}"
            )
            self.finalize_log('UNKNOWN')

        # Move to next goal after delay
        self._next_goal_timer = self.create_timer(
            self.delay_between_goals, self._next_goal_timer_callback
        )

    def _next_goal_timer_callback(self):
        """One-shot callback to move to next goal."""
        self.destroy_timer(self._next_goal_timer)
        self.move_to_next_goal()

    def move_to_next_goal(self):
        """Increment goal index and start next goal."""
        self.current_goal_index += 1
        self.start_next_goal()

    def scan_callback(self, msg):
        if not self.is_navigating:
            return
        valid_ranges = [
            r for r in msg.ranges if msg.range_min < r < msg.range_max
        ]
        if valid_ranges:
            min_dist = min(valid_ranges)
            if min_dist < self.min_obstacle_dist:
                self.min_obstacle_dist = min_dist

            if min_dist < 0.20:
                self.near_collision_count += 1

    def odom_callback(self, msg):
        if not self.is_navigating:
            return

        current_time = self.get_clock().now()
        current_pos = msg.pose.pose.position

        # Distance traveled
        if self.last_odom_pos:
            dx = current_pos.x - self.last_odom_pos.x
            dy = current_pos.y - self.last_odom_pos.y
            dist = math.sqrt(dx*dx + dy*dy)
            self.distance_traveled += dist
        self.last_odom_pos = current_pos

        # Dynamics
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        w_z = msg.twist.twist.angular.z

        linear_vel = math.sqrt(v_x**2 + v_y**2)
        lateral_vel = abs(v_y)

        self.velocities_linear.append(linear_vel)
        self.velocities_angular.append(abs(w_z))
        self.velocities_lateral.append(lateral_vel)

        # Acceleration and jerk
        if self.last_odom_time:
            dt = (current_time - self.last_odom_time).nanoseconds / 1e9
            if dt > 0:
                accel = (linear_vel - self.last_vel_linear) / dt
                self.accels_linear.append(abs(accel))

                jerk = (accel - self.last_accel_linear) / dt
                self.jerks_linear.append(abs(jerk))

                self.last_accel_linear = accel

        self.last_vel_linear = linear_vel
        self.last_odom_time = current_time

        # System resources
        self.cpu_usages.append(psutil.cpu_percent(interval=None))
        self.ram_usages.append(
            psutil.virtual_memory().used / (1024*1024)
        )

    def plan_callback(self, msg):
        if not self.is_navigating:
            return

        if self.planning_start_time and not self.planning_end_time:
            self.planning_end_time = self.get_clock().now()

        # Calculate path smoothness
        smoothness = 0.0
        poses = msg.poses
        for i in range(1, len(poses)-1):
            p_prev = poses[i-1].pose.position
            p_curr = poses[i].pose.position
            p_next = poses[i+1].pose.position

            angle1 = math.atan2(
                p_curr.y - p_prev.y, p_curr.x - p_prev.x
            )
            angle2 = math.atan2(
                p_next.y - p_curr.y, p_next.x - p_curr.x
            )
            diff = abs(angle2 - angle1)
            if diff > math.pi:
                diff = 2*math.pi - diff
            smoothness += diff

        self.path_smoothness = smoothness

    def finalize_log(self, outcome):
        if not self.start_time:
            return
        end_time = self.get_clock().now()
        total_time = (end_time - self.start_time).nanoseconds / 1e9

        plan_time = 0.0
        if self.planning_end_time:
            plan_time = (
                (self.planning_end_time - self.planning_start_time)
                .nanoseconds / 1e9
            )

        # Calculate averages and maxes
        avg_lin_vel = (
            np.mean(self.velocities_linear)
            if self.velocities_linear else 0.0
        )
        max_lin_vel = (
            np.max(self.velocities_linear)
            if self.velocities_linear else 0.0
        )

        avg_ang_vel = (
            np.mean(self.velocities_angular)
            if self.velocities_angular else 0.0
        )
        max_ang_vel = (
            np.max(self.velocities_angular)
            if self.velocities_angular else 0.0
        )

        avg_lat_vel = (
            np.mean(self.velocities_lateral)
            if self.velocities_lateral else 0.0
        )
        max_lat_vel = (
            np.max(self.velocities_lateral)
            if self.velocities_lateral else 0.0
        )

        max_accel = (
            np.max(self.accels_linear) if self.accels_linear else 0.0
        )
        max_jerk = (
            np.max(self.jerks_linear) if self.jerks_linear else 0.0
        )

        avg_cpu = (
            np.mean(self.cpu_usages) if self.cpu_usages else 0.0
        )
        max_ram = (
            np.max(self.ram_usages) if self.ram_usages else 0.0
        )

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Get current goal info
        goal_config = self.goal_poses[self.current_goal_index]

        row = [
            self.controller_name, self.planner_name,
            self.current_goal_index,
            f"{goal_config['x']:.3f}", f"{goal_config['y']:.3f}",
            f"{goal_config['yaw']:.3f}",
            timestamp, outcome, f"{total_time:.3f}", f"{plan_time:.3f}",
            f"{self.distance_traveled:.3f}", f"{self.path_smoothness:.3f}",
            f"{avg_lin_vel:.3f}", f"{max_lin_vel:.3f}",
            f"{avg_ang_vel:.3f}", f"{max_ang_vel:.3f}",
            f"{avg_lat_vel:.3f}", f"{max_lat_vel:.3f}",
            f"{max_accel:.3f}", f"{max_jerk:.3f}",
            f"{self.min_obstacle_dist:.3f}", self.near_collision_count,
            f"{avg_cpu:.1f}", f"{max_ram:.1f}", self.recovery_count
        ]

        with open(self.csv_file_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)

        self.get_logger().info(
            f"Logged Goal {self.current_goal_index}: {outcome} | "
            f"Time: {total_time:.2f}s | "
            f"Distance: {self.distance_traveled:.2f}m | "
            f"MaxJerk: {max_jerk:.2f}"
        )
        self.reset_metrics()


def main(args=None):
    rclpy.init(args=args)
    node = NavMetricsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import math
import csv
import os
import psutil
from datetime import datetime
import numpy as np


class NavMetricsLoggerStandalone(Node):
    def __init__(self):
        super().__init__('nav_metrics_logger_sa')

        # Controller and planner names for logging
        self.controller_name = 'MPPI'
        self.planner_name = 'ThetaStar'

        # Velocity threshold to detect when navigation starts/stops
        self.velocity_threshold = 0.05  # m/s
        self.idle_timeout = 3.0  # seconds of low velocity

        # CSV output path - will be created in the eval directory
        try:
            from ament_index_python.packages import (
                get_package_share_directory
            )
            pkg_share_dir = get_package_share_directory(
                'skratch_navigation'
            )
            workspace_root = os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    pkg_share_dir
                )))
            )
            src_eval_dir = os.path.join(
                workspace_root, 'src', 'skratch_navigation', 'eval'
            )
            self.csv_file_path = os.path.join(
                src_eval_dir, 'nav_metrics_standalone.csv'
            )
        except Exception:
            # Fallback: use relative path from this file
            src_pkg_dir = os.path.dirname(os.path.dirname(__file__))
            self.csv_file_path = os.path.join(
                src_pkg_dir, 'eval', 'nav_metrics_standalone.csv'
            )

        self.trial_count = 0
        self.last_activity_time = None

        # Initialize CSV
        self.init_csv()

        # State Variables
        self.reset_metrics()

        # Subscriptions for metrics
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(
            LaserScan, '/scan_combined', self.scan_callback, 10
        )

        # Timer to check for navigation completion
        self.check_timer = self.create_timer(
            0.5, self.check_navigation_state
        )

        self.get_logger().info("Nav Metrics Logger (Passive) Started")
        self.get_logger().info(f"Logging to: {self.csv_file_path}")
        self.get_logger().info(
            f"Controller: {self.controller_name}, "
            f"Planner: {self.planner_name}"
        )
        self.get_logger().info("Waiting for navigation activity...")

    def init_csv(self):
        headers = [
            'Trial', 'Controller', 'Planner',
            'Timestamp', 'Total_Time(s)', 'Planning_Time(s)',
            'Path_Length(m)', 'Path_Smoothness(rad)',
            'Avg_Linear_Vel(m/s)', 'Max_Linear_Vel(m/s)',
            'Avg_Angular_Vel(rad/s)', 'Max_Angular_Vel(rad/s)',
            'Avg_Lateral_Vel(m/s)', 'Max_Lateral_Vel(m/s)',
            'Max_Accel_Linear(m/s2)', 'Max_Jerk_Linear(m/s3)',
            'Min_Obstacle_Dist(m)', 'Near_Collisions_Count',
            'Avg_CPU_Usage(%)', 'Max_RAM_Usage(MB)'
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

    def check_navigation_state(self):
        """Check if navigation has completed based on inactivity."""
        if not self.is_navigating or not self.last_activity_time:
            return

        current_time = self.get_clock().now()
        idle_duration = (
            (current_time - self.last_activity_time).nanoseconds / 1e9
        )

        if idle_duration > self.idle_timeout:
            self.get_logger().info(
                f"Navigation completed (idle for {idle_duration:.1f}s)"
            )
            self.finalize_log()

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
        current_time = self.get_clock().now()
        current_pos = msg.pose.pose.position

        # Dynamics
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        w_z = msg.twist.twist.angular.z

        linear_vel = math.sqrt(v_x**2 + v_y**2)
        lateral_vel = abs(v_y)

        # Detect start of navigation
        if not self.is_navigating and linear_vel > self.velocity_threshold:
            self.get_logger().info(
                f"Navigation started (Trial {self.trial_count + 1})"
            )
            self.reset_metrics()
            self.is_navigating = True
            self.start_time = current_time
            self.planning_start_time = current_time
            self.last_activity_time = current_time

        if not self.is_navigating:
            return

        # Update activity time if robot is moving
        if linear_vel > self.velocity_threshold:
            self.last_activity_time = current_time

        # Distance traveled
        if self.last_odom_pos:
            dx = current_pos.x - self.last_odom_pos.x
            dy = current_pos.y - self.last_odom_pos.y
            dist = math.sqrt(dx*dx + dy*dy)
            self.distance_traveled += dist
        self.last_odom_pos = current_pos

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

    def finalize_log(self):
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

        self.trial_count += 1

        row = [
            self.trial_count,
            self.controller_name, self.planner_name,
            timestamp, f"{total_time:.3f}", f"{plan_time:.3f}",
            f"{self.distance_traveled:.3f}", f"{self.path_smoothness:.3f}",
            f"{avg_lin_vel:.3f}", f"{max_lin_vel:.3f}",
            f"{avg_ang_vel:.3f}", f"{max_ang_vel:.3f}",
            f"{avg_lat_vel:.3f}", f"{max_lat_vel:.3f}",
            f"{max_accel:.3f}", f"{max_jerk:.3f}",
            f"{self.min_obstacle_dist:.3f}", self.near_collision_count,
            f"{avg_cpu:.1f}", f"{max_ram:.1f}"
        ]

        with open(self.csv_file_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)

        self.get_logger().info(
            f"✓ Logged Trial {self.trial_count}: "
            f"Time={total_time:.2f}s | "
            f"Distance={self.distance_traveled:.2f}m | "
            f"MaxJerk={max_jerk:.2f}"
        )
        self.get_logger().info("Waiting for next navigation activity...")
        self.reset_metrics()


def main(args=None):
    rclpy.init(args=args)
    node = NavMetricsLoggerStandalone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

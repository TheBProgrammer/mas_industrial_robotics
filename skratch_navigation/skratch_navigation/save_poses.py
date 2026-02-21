#!/usr/bin/env python3
"""
Interactive tool to save robot poses to a YAML file.
Captures current robot transform (map -> base_link) and saves as [x, y, yaw].
"""

import sys
import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import (
    LookupException, ConnectivityException, ExtrapolationException
)
import yaml
import math
import threading
from rclpy.duration import Duration
from rclpy.utilities import remove_ros_args


class SavePosesToFile(Node):
    def __init__(self, file_name='navigation_goals.yaml'):
        super().__init__('save_poses_to_file')

        # Declare parameters
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')

        # Get parameter values
        self.target_frame = (
            self.get_parameter('target_frame')
            .get_parameter_value().string_value
        )
        self.source_frame = (
            self.get_parameter('source_frame')
            .get_parameter_value().string_value
        )

        # Get package share directory path
        self.package_name = 'skratch_navigation'
        self.file_path = self.get_save_file_path(file_name)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Save Poses Node initialized')
        self.get_logger().info(
            f'Target Frame: {self.target_frame}, '
            f'Source Frame: {self.source_frame}'
        )
        self.get_logger().info(f'Poses will be saved to: {self.file_path}')

    def get_save_file_path(self, file_name):
        """Get the full path to save the YAML file in config directory."""
        try:
            # Get the workspace source directory
            ws_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '../../..')
            )
            config_path = os.path.join(
                ws_path, 'src', self.package_name, 'config', file_name
            )

            # Ensure config directory exists
            os.makedirs(os.path.dirname(config_path), exist_ok=True)

            return config_path
        except Exception:
            self.get_logger().error(
                'Error determining save path'
            )
            # Fallback to current directory
            return file_name

    def wait_for_transforms(self, timeout=10.0):
        """Wait for required transforms to be available."""
        self.get_logger().info('Waiting for TF buffer to populate...')

        # We rely on the background thread to populate the buffer now
        # Just wait for a valid timestamp
        start_time = self.get_clock().now()
        while start_time.nanoseconds == 0 and rclpy.ok():
            self.get_logger().info(
                'Waiting for valid clock time...',
                throttle_duration_sec=2.0
            )
            # Short sleep to allow background thread to update clock
            import time
            time.sleep(0.1)
            start_time = self.get_clock().now()

        self.get_logger().info(f'Start Time: {start_time.nanoseconds}')

        while rclpy.ok():
            try:
                # Try to get the transform
                if self.tf_buffer.can_transform(
                    self.target_frame, self.source_frame,
                    rclpy.time.Time()
                ):
                    self.get_logger().info(
                        f'Transform {self.target_frame} -> '
                        f'{self.source_frame} is available'
                    )
                    return True
            except Exception:
                pass

            # Check timeout
            now = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().info(
                    f'Current Time: {now.nanoseconds}, '
                    f'Elapsed: {elapsed}'
                )
                self.get_logger().error(
                    f'Timeout waiting for transforms after {timeout}s'
                )
                self.get_logger().error(
                    f'Could not transform from {self.source_frame} '
                    f'to {self.target_frame}'
                )
                self.get_logger().error('Possible causes:')
                self.get_logger().error(
                    '1. Nav2 or Localization is not running'
                )
                self.get_logger().error(
                    '2. Time Source Mismatch (Wall Time vs Sim Time)'
                )
                self.get_logger().error(
                    '   Try running with: ros2 run skratch_navigation '
                    'save_poses --ros-args -p use_sim_time:=true'
                )
                return False

            # Spin is handled by background thread now
            import time
            time.sleep(0.1)

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

    def get_current_pose(self):
        """Get current robot pose from TF."""
        try:
            # Look up the latest transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract yaw from quaternion
            yaw = self.quaternion_to_yaw(transform.transform.rotation)

            self.get_logger().info('Transform received successfully')

            return [x, y, yaw]

        except LookupException as e:
            self.get_logger().error(f'ERROR: Transform lookup failed: {e}')
            self.get_logger().error(
                f'   This usually means the "{self.target_frame}" '
                'frame is not being published.'
            )
            self.get_logger().error(
                '   Start Nav2 with localization or SLAM first!'
            )
            return None
        except ConnectivityException as e:
            self.get_logger().error(f'ERROR: TF connectivity error: {e}')
            self.get_logger().error(
                f'   The transform chain from {self.target_frame} to '
                f'{self.source_frame} is broken.'
            )
            return None
        except ExtrapolationException as e:
            self.get_logger().error(f'ERROR: TF extrapolation error: {e}')
            self.get_logger().error(
                '   Transform data is too old or not available yet.'
            )
            return None
        except Exception as e:
            self.get_logger().error(f'ERROR: Unexpected error: {e}')
            return None

    def save_pose(self, pose_name, pose_data):
        """Save pose to YAML file."""
        # Load existing data if file exists
        poses = {}
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    poses = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().warn(
                    f'Could not read existing file: {e}'
                )

        # Add new pose
        poses[pose_name] = pose_data

        # Write to file
        try:
            with open(self.file_path, 'w') as f:
                yaml.dump(poses, f, default_flow_style=False)
            self.get_logger().info(
                f'Saved pose "{pose_name}" to {self.file_path}'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save to file: {e}')
            return False

    def run_interactive_loop(self):
        """Run interactive loop to save poses."""
        # Start background thread for spinning
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True
        )
        spin_thread.start()

        # Wait for transforms to be available first
        if not self.wait_for_transforms(timeout=15.0):
            self.get_logger().error('Failed to initialize. Exiting.')
            return

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Interactive Pose Saving Tool')
        self.get_logger().info('='*60)
        self.get_logger().info('Instructions:')
        self.get_logger().info('  - Move your robot to desired position')
        self.get_logger().info('  - Enter a name for the pose')
        self.get_logger().info('  - Type "quit" or "exit" to finish')
        self.get_logger().info('='*60 + '\n')

        while rclpy.ok():
            try:
                # Get pose name from user
                pose_name = input(
                    "\nEnter pose name (or 'quit' to exit): "
                ).strip()

                # Check for exit condition
                if pose_name.lower() in ['quit', 'exit', 'q', '']:
                    self.get_logger().info('Exiting...')
                    break

                # Get current pose
                pose_data = self.get_current_pose()

                if pose_data is not None:
                    # Display pose info
                    self.get_logger().info(
                        f'Current pose: x={pose_data[0]:.3f}, '
                        f'y={pose_data[1]:.3f}, yaw={pose_data[2]:.3f}'
                    )

                    # Save to file
                    if self.save_pose(pose_name, pose_data):
                        self.get_logger().info(
                            f'Pose "{pose_name}" saved successfully!'
                        )
                else:
                    self.get_logger().error(
                        'Could not get current pose. '
                        'Make sure TF is published.'
                    )

            except KeyboardInterrupt:
                self.get_logger().info(
                    '\nInterrupted by user. Exiting...'
                )
                break
            except Exception as e:
                self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)

    # Parse args properly
    clean_args = remove_ros_args(args=sys.argv)

    # Check for custom file name argument
    file_name = 'navigation_goals.yaml'
    if len(clean_args) > 1:
        arg = clean_args[1]
        file_name = arg if arg.endswith('.yaml') else arg + '.yaml'

    node = SavePosesToFile(file_name)

    try:
        # Run interactive loop
        node.run_interactive_loop()
    except Exception as e:
        node.get_logger().error(f'Fatal error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import tf2_ros

from std_srvs.srv import Trigger

import yaml
import math
import os


class SaveRobotPose(Node):
    def __init__(self):
        super().__init__('save_robot_pose')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Service to trigger pose saving
        self.save_service = self.create_service(
            Trigger,
            'save_pose',
            self.save_pose_callback
        )

        # File to store poses
        self.yaml_file = os.path.join(
            os.path.dirname(__file__),
            'saved_poses.yaml'
        )

        self.get_logger().info('SaveRobotPose node ready. Call /save_pose to store robot pose.')

    def save_pose_callback(self, request, response):
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            theta = self.quaternion_to_yaw(q)

            pose_data = {
                'x': float(x),
                'y': float(y),
                'theta': float(theta)
            }

            self.write_pose_to_yaml(pose_data)

            response.success = True
            response.message = f"Pose saved: x={x:.2f}, y={y:.2f}, θ={theta:.2f}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(response.message)

        return response

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def write_pose_to_yaml(self, pose):
        # Load existing data if file exists
        if os.path.exists(self.yaml_file):
            with open(self.yaml_file, 'r') as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        pose_id = f"pose_{len(data) + 1}"
        data[pose_id] = pose

        with open(self.yaml_file, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f"Pose stored as {pose_id} in {self.yaml_file}")


def main(args=None):
    rclpy.init(args=args)
    node = SaveRobotPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Mock barrier tape detection node.

Publishes a fixed PointCloud2 representing barrier tape points
in the map frame. The tape coordinates are runtime-configurable
via ROS2 dynamic parameters.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class BarrierTapeMockNode(Node):
    """Publishes a PointCloud2 line segment simulating barrier tape."""

    def __init__(self):
        """Initialize the barrier tape mock node."""
        super().__init__('barrier_tape_mock')

        # Declare parameters
        self.declare_parameter(
            'tape_start_x', 4.0,
            ParameterDescriptor(description='Tape start X in map frame (m)'))
        self.declare_parameter(
            'tape_start_y', 1.5,
            ParameterDescriptor(description='Tape start Y in map frame (m)'))
        self.declare_parameter(
            'tape_end_x', 4.0,
            ParameterDescriptor(description='Tape end X in map frame (m)'))
        self.declare_parameter(
            'tape_end_y', 3.0,
            ParameterDescriptor(description='Tape end Y in map frame (m)'))
        self.declare_parameter(
            'num_points', 40,
            ParameterDescriptor(description='Number of points along the tape'))
        self.declare_parameter(
            'publish_rate', 10.0,
            ParameterDescriptor(description='Publish rate in Hz'))
        self.declare_parameter(
            'frame_id', 'map',
            ParameterDescriptor(description='Frame ID for the pointcloud'))
        self.declare_parameter(
            'clear_history', False,
            ParameterDescriptor(
                description='Set true to wipe all accumulated tape history'))

        # All tape points ever added this session (survives costmap clears)
        self._all_points = np.empty((0, 3), dtype=np.float32)

        # Read initial values
        self._read_parameters()

        # Publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/barrier_tape/pointcloud', 10)

        # Timer
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self._publish_cloud)

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.get_logger().info(
            f'Barrier tape mock started: '
            f'({self.start_x:.2f}, {self.start_y:.2f}) -> '
            f'({self.end_x:.2f}, {self.end_y:.2f}), '
            f'{self.num_points} pts @ {rate} Hz in '
            f'"{self.frame_id}" frame')

    def _read_parameters(self):
        """Read all tape parameters from the parameter server."""
        self.start_x = self.get_parameter('tape_start_x').value
        self.start_y = self.get_parameter('tape_start_y').value
        self.end_x = self.get_parameter('tape_end_x').value
        self.end_y = self.get_parameter('tape_end_y').value
        self.num_points = self.get_parameter('num_points').value
        self.frame_id = self.get_parameter('frame_id').value
        self._rebuild_cloud_data()

    def _rebuild_cloud_data(self):
        """Append new tape segment to accumulated history and repack bytes."""
        xs = np.linspace(self.start_x, self.end_x, self.num_points,
                         dtype=np.float32)
        ys = np.linspace(self.start_y, self.end_y, self.num_points,
                         dtype=np.float32)
        zs = np.zeros(self.num_points, dtype=np.float32)

        # Accumulate: append new segment, keep all prior history
        new_pts = np.column_stack((xs, ys, zs))
        self._all_points = np.vstack((self._all_points, new_pts))
        self._cloud_bytes = self._all_points.tobytes()

    def _on_parameter_change(self, params):
        """Handle dynamic parameter updates."""
        for param in params:
            if param.name == 'tape_start_x':
                self.start_x = param.value
            elif param.name == 'tape_start_y':
                self.start_y = param.value
            elif param.name == 'tape_end_x':
                self.end_x = param.value
            elif param.name == 'tape_end_y':
                self.end_y = param.value
            elif param.name == 'num_points':
                self.num_points = param.value
            elif param.name == 'frame_id':
                self.frame_id = param.value
            elif param.name == 'publish_rate':
                # Recreate the timer with new rate
                self.timer.cancel()
                self.timer = self.create_timer(
                    1.0 / param.value, self._publish_cloud)
            elif param.name == 'clear_history' and param.value is True:
                self._all_points = np.empty((0, 3), dtype=np.float32)
                self._cloud_bytes = b''
                self.get_logger().info('Tape history cleared.')
                # Reset the param back to false
                self.set_parameters([rclpy.parameter.Parameter(
                    'clear_history', rclpy.parameter.Parameter.Type.BOOL,
                    False)])

        self._rebuild_cloud_data()
        self.get_logger().info(
            f'Tape updated: ({self.start_x:.2f}, {self.start_y:.2f}) -> '
            f'({self.end_x:.2f}, {self.end_y:.2f}), {self.num_points} pts')
        return SetParametersResult(successful=True)

    def _publish_cloud(self):
        """Publish the barrier tape as a PointCloud2 message."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(self._all_points)

        # XYZ float32 fields
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * len(self._all_points)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.data = self._cloud_bytes

        self.cloud_pub.publish(msg)


def main(args=None):
    """Run the barrier tape mock node."""
    rclpy.init(args=args)
    node = BarrierTapeMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

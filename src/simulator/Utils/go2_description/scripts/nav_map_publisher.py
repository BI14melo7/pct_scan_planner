#!/usr/bin/python3
"""Publish the point-cloud twin of worlds/navigation.sdf for SCAN-Planner."""

import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


# Every tuple is (center_x, center_y, size_x, size_y, height).
# Keep this list synchronized with the boxes in worlds/navigation.sdf.
BOXES = (
    (0.0, 8.0, 20.0, 0.20, 2.0),
    (0.0, -8.0, 20.0, 0.20, 2.0),
    (10.0, 0.0, 0.20, 16.0, 2.0),
    (-10.0, 0.0, 0.20, 16.0, 2.0),
    (0.0, 0.0, 1.0, 6.0, 1.8),
    (-4.0, 2.5, 2.0, 2.0, 1.5),
    (4.0, -2.5, 2.0, 2.0, 1.5),
    (-5.5, -3.0, 1.2, 1.2, 1.8),
    (5.5, 3.0, 1.2, 1.2, 1.8),
)


def sample_axis(start, stop, resolution):
    count = max(1, int(math.ceil((stop - start) / resolution)))
    return [start + (stop - start) * index / count for index in range(count + 1)]


def sample_box(cx, cy, sx, sy, height, resolution=0.10):
    """Sample vertical faces and top surface of an axis-aligned obstacle."""
    xs = sample_axis(cx - sx / 2.0, cx + sx / 2.0, resolution)
    ys = sample_axis(cy - sy / 2.0, cy + sy / 2.0, resolution)
    zs = sample_axis(0.0, height, resolution)
    points = set()
    for z in zs:
        for x in xs:
            points.add((x, cy - sy / 2.0, z))
            points.add((x, cy + sy / 2.0, z))
        for y in ys:
            points.add((cx - sx / 2.0, y, z))
            points.add((cx + sx / 2.0, y, z))
    for x in xs:
        for y in ys:
            points.add((x, y, height))
    return points


class NavigationMapPublisher(Node):
    def __init__(self):
        super().__init__('go2_navigation_map')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PointCloud2, '/map_generator/global_cloud', qos)
        points = set()
        for box in BOXES:
            points.update(sample_box(*box))
        self.points = sorted(points)
        self.timer = self.create_timer(2.0, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f'Navigation map ready: {len(self.points)} points, frame=world')

    def publish_map(self):
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'
        message.height = 1
        message.width = len(self.points)
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b''.join(struct.pack('<fff', *point) for point in self.points)
        message.is_dense = True
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationMapPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

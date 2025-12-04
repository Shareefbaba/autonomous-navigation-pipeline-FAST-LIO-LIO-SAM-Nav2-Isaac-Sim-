#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2
import inspect


class RingConverterNode(Node):
    def __init__(self):
        super().__init__("ring_converter")

        # Just to be 100% sure we're running THIS file:
        self.get_logger().warn(
            f"### USING TIME+INTENSITY VERSION, file = {inspect.getfile(RingConverterNode)} ###"
        )

        # Parameters
        self.declare_parameter("input_topic", "/point_cloud_3d/point_cloud")
        self.declare_parameter("output_topic", "/points_with_ring")
        self.declare_parameter("n_scan", 32)
        # seconds per full scan, e.g. 0.1 for 10 Hz lidar
        self.declare_parameter("scan_period", 0.1)
        # default intensity value to fill in (since IsaacSim has none)
        self.declare_parameter("default_intensity", 1.0)

        self.input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.n_scan = (
            self.get_parameter("n_scan").get_parameter_value().integer_value
        )
        self.scan_period = (
            self.get_parameter("scan_period").get_parameter_value().double_value
        )
        self.default_intensity = (
            self.get_parameter("default_intensity").get_parameter_value().double_value
        )

        if self.n_scan <= 0:
            self.get_logger().warn(
                f"n_scan was {self.n_scan}, resetting to 32"
            )
            self.n_scan = 32

        if self.scan_period <= 0.0:
            self.get_logger().warn(
                f"scan_period was {self.scan_period}, resetting to 0.1"
            )
            self.scan_period = 0.1

        self.get_logger().info(
            f"RingConverterNode: input={self.input_topic}, "
            f"output={self.output_topic}, n_scan={self.n_scan}, "
            f"scan_period={self.scan_period}s, "
            f"default_intensity={self.default_intensity}"
        )

        # Publisher & subscriber
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, 10
        )

    def cloud_callback(self, msg: PointCloud2):
        """Convert xyz-only cloud to xyz+intensity+ring+time cloud."""

        # Read original points (x, y, z) and skip NaNs
        points_iter = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )

        # Convert iterator to list so we know total number of points
        points_xyz = list(points_iter)
        total_points = len(points_xyz)

        if total_points == 0:
            self.get_logger().warn(
                "No valid points after NaN filtering, skipping frame."
            )
            return

        points_out = []
        # Approximate firing time: spread points evenly over [0, scan_period]
        denom = max(1, total_points - 1)

        for idx, (x, y, z) in enumerate(points_xyz):
            ring = idx % self.n_scan          # simple fake ring assignment
            rel_time = (idx / denom) * self.scan_period  # seconds within scan
            intensity = self.default_intensity
            # x, y, z, intensity, ring, time
            points_out.append([x, y, z, intensity, ring, rel_time])

        # Define new fields:
        #  x, y, z         : float32
        #  intensity       : float32
        #  ring            : uint16
        #  time            : float32
        #
        # Layout (offsets in bytes):
        #  x         @ 0
        #  y         @ 4
        #  z         @ 8
        #  intensity @ 12
        #  ring      @ 16  (2 bytes, so next field at 20)
        #  time      @ 20
        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring",      offset=16, datatype=PointField.UINT16,  count=1),
            PointField(name="time",      offset=20, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id  # keep same frame as original

        new_cloud = pc2.create_cloud(header, fields, points_out)
        new_cloud.is_dense = True

        self.pub.publish(new_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = RingConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

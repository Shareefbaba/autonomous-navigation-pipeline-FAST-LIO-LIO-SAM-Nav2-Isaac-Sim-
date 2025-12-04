#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class FakeImuFromOdom(Node):
    def __init__(self):
        super().__init__("fake_imu_from_odom")

        # Subscribe to your odom topic from Isaac
        self.odom_sub = self.create_subscription(
            Odometry,
            "/chassis/odom",        # <--- your odom topic
            self.odom_callback,
            50,
        )

        # Publish fake IMU here (match LIO-SAM imuTopic)
        self.imu_pub = self.create_publisher(Imu, "/chassis/imu_fake", 100)

        # Previous state for finite differences
        self.prev_v = np.zeros(3)
        self.prev_yaw = None
        self.prev_t = None

        # Gravity value from your params
        self.g = 9.80511

        self.get_logger().info(
            "Fake IMU from /chassis/odom started, publishing /chassis/imu_fake"
        )

    def odom_callback(self, odom: Odometry):
        # Time in seconds
        t = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9

        # Linear velocity from odom (m/s)
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z
        v = np.array([vx, vy, vz])

        # Orientation (quaternion)
        q = odom.pose.pose.orientation
        # Approx yaw from quaternion (assuming z-up world)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if self.prev_t is None:
            # First callback: initialize history and return
            self.prev_t = t
            self.prev_v = v
            self.prev_yaw = yaw
            return

        dt = t - self.prev_t
        if dt <= 0.0:
            return

        # -------- Linear acceleration (world frame, no gravity yet) --------
        a_world = (v - self.prev_v) / dt

        # Gravity in world frame (assuming +Z up, gravity down)
        g_world = np.array([0.0, 0.0, -self.g])

        # Total accel including gravity
        a_world_with_g = a_world + g_world

        # For now, assume IMU frame == odom/world frame
        lin_acc_imu = a_world_with_g

        # -------- Angular velocity around Z from yaw rate --------
        dyaw = yaw - self.prev_yaw
        # unwrap to [-pi, pi]
        if dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2.0 * math.pi
        wz = dyaw / dt

        # -------- Fill IMU message --------
        imu = Imu()
        imu.header.stamp = odom.header.stamp
        imu.header.frame_id = "Imu_Sensor"  # same as your current IMU frame

        # Angular velocity (rad/s)
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = float(wz)

        # Linear acceleration (m/s^2)
        imu.linear_acceleration.x = float(lin_acc_imu[0])
        imu.linear_acceleration.y = float(lin_acc_imu[1])
        imu.linear_acceleration.z = float(lin_acc_imu[2])

        # Orientation: reuse odom orientation
        imu.orientation = odom.pose.pose.orientation

        self.imu_pub.publish(imu)

        # -------- Update history --------
        self.prev_t = t
        self.prev_v = v
        self.prev_yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = FakeImuFromOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time as TimeMsg

class ImuUpsampler(Node):
    def __init__(self):
        super().__init__('imu_upsampler')

        # Parameters
        self.declare_parameter('input_topic', '/chassis/imu')
        self.declare_parameter('output_topic', '/chassis/imu_upsampled')
        self.declare_parameter('upsample_factor', 10)  # 20 Hz * 10 = 200 Hz

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.upsample_factor = self.get_parameter('upsample_factor').get_parameter_value().integer_value

        if self.upsample_factor < 1:
            self.get_logger().warn('upsample_factor < 1, setting to 1')
            self.upsample_factor = 1

        self.get_logger().info(
            f'IMU upsampler started: {self.input_topic} -> {self.output_topic}, '
            f'factor = {self.upsample_factor}'
        )

        self.sub = self.create_subscription(Imu, self.input_topic, self.imu_callback, 100)
        self.pub = self.create_publisher(Imu, self.output_topic, 100)

        self.prev_msg = None

    def imu_callback(self, msg: Imu):
        # First message: just forward and store
        if self.prev_msg is None:
            self.pub.publish(msg)
            self.prev_msg = msg
            return

        # Time difference between previous and current IMU
        t0 = self.prev_msg.header.stamp
        t1 = msg.header.stamp
        dt_sec = (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9

        if dt_sec <= 0.0 or self.upsample_factor == 1:
            # Something odd with timestamps or no upsampling requested
            self.pub.publish(msg)
            self.prev_msg = msg
            return

        # We will generate (upsample_factor - 1) extra messages between t0 and t1
        # using the *previous* IMU values (piecewise constant model).
        dt_step = dt_sec / float(self.upsample_factor)

        for i in range(1, self.upsample_factor):
            t_interp = self.add_dt_to_time(t0, dt_step * i)

            up_msg = Imu()
            # Copy data from prev_msg (piecewise constant approximation)
            up_msg.header = self.prev_msg.header
            up_msg.header.stamp = t_interp

            up_msg.orientation = self.prev_msg.orientation
            up_msg.orientation_covariance = self.prev_msg.orientation_covariance

            up_msg.angular_velocity = self.prev_msg.angular_velocity
            up_msg.angular_velocity_covariance = self.prev_msg.angular_velocity_covariance

            up_msg.linear_acceleration = self.prev_msg.linear_acceleration
            up_msg.linear_acceleration_covariance = self.prev_msg.linear_acceleration_covariance

            self.pub.publish(up_msg)

        # Finally publish the real incoming message
        self.pub.publish(msg)

        # Update previous message
        self.prev_msg = msg

    @staticmethod
    def add_dt_to_time(t: TimeMsg, dt: float) -> TimeMsg:
        """Return a new TimeMsg = t + dt seconds."""
        total_nsec = t.sec * 1_000_000_000 + t.nanosec + int(dt * 1_000_000_000)
        new_sec = total_nsec // 1_000_000_000
        new_nsec = total_nsec % 1_000_000_000
        out = TimeMsg()
        out.sec = int(new_sec)
        out.nanosec = int(new_nsec)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ImuUpsampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

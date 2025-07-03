import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from builtin_interfaces.msg import Time
from collections import deque
import numpy as np

class IMUMerger(Node):
    def __init__(self):
        super().__init__('imu_merger')

        self.accel_buffer = deque(maxlen=100)
        self.gyro_buffer = deque(maxlen=100)

        self.pub = self.create_publisher(Imu, '/camera/realsense2_camera/imu', 10)

        self.sub_accel = self.create_subscription(Vector3Stamped, '/t265/accel/sample', self.accel_callback, 10)
        self.sub_gyro = self.create_subscription(Vector3Stamped, '/t265/gyro/sample', self.gyro_callback, 10)

        self.get_logger().info("IMU Merger node running...")

    def accel_callback(self, msg):
        self.accel_buffer.append(msg)
        self.try_publish(msg.header.stamp)

    def gyro_callback(self, msg):
        self.gyro_buffer.append(msg)
        self.try_publish(msg.header.stamp)

    def try_publish(self, stamp: Time):
        accel = self.find_closest(self.accel_buffer, stamp)
        gyro = self.find_closest(self.gyro_buffer, stamp)

        if accel is None or gyro is None:
            return  # Wait for matching timestamps

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"

        imu.linear_acceleration = accel.vector
        imu.angular_velocity = gyro.vector

        # Leave orientation unset or zeroed
        imu.orientation_covariance[0] = -1  # Indicates no orientation

        self.pub.publish(imu)

    def find_closest(self, buffer, target_stamp):
        # Find msg in buffer with closest timestamp
        if not buffer:
            return None
        best = min(buffer, key=lambda msg: abs(self.to_sec(msg.header.stamp) - self.to_sec(target_stamp)))
        if abs(self.to_sec(best.header.stamp) - self.to_sec(target_stamp)) < 0.01:  # within 10ms
            return best
        return None

    def to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

def main(args=None):
    rclpy.init(args=args)
    node = IMUMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

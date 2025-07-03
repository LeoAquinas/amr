import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from collections import deque

class ImuMerger(Node):
    def __init__(self):
        super().__init__('imu_merger')

        # Buffers to store incoming accel and gyro messages
        self.accel_msgs = deque(maxlen=100)
        self.gyro_msgs = deque(maxlen=100)

        self.accel_sub = self.create_subscription(Imu, '/t265/accel/sample', self.accel_callback, 10)
        self.gyro_sub = self.create_subscription(Imu, '/t265/gyro/sample', self.gyro_callback, 10)

        self.imu_pub = self.create_publisher(Imu, '/camera/realsense2_camera/imu', 10)

        self.get_logger().info("IMU merger initialized.")

    def accel_callback(self, msg):
        self.accel_msgs.append(msg)
        self.try_merge(msg.header.stamp)

    def gyro_callback(self, msg):
        self.gyro_msgs.append(msg)
        self.try_merge(msg.header.stamp)

    def try_merge(self, stamp):
        accel_msg = self.find_closest(self.accel_msgs, stamp)
        gyro_msg = self.find_closest(self.gyro_msgs, stamp)

        if accel_msg is None or gyro_msg is None:
            return

        # Merge IMU data
        merged_imu = Imu()
        merged_imu.header.stamp = stamp
        merged_imu.header.frame_id = 'imu_link'

        merged_imu.linear_acceleration = accel_msg.linear_acceleration
        merged_imu.angular_velocity = gyro_msg.angular_velocity

        merged_imu.orientation_covariance[0] = -1  # Means: orientation is not provided

        self.imu_pub.publish(merged_imu)

    def find_closest(self, buffer, target_stamp):
        if not buffer:
            return None
        closest = min(buffer, key=lambda m: abs(self.to_sec(m.header.stamp) - self.to_sec(target_stamp)))
        if abs(self.to_sec(closest.header.stamp) - self.to_sec(target_stamp)) < 0.01:  # 10ms threshold
            return closest
        return None

    def to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

def main(args=None):
    rclpy.init(args=args)
    node = ImuMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

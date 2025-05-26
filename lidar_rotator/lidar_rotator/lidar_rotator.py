#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ScanRotator(Node):
    def __init__(self):
        super().__init__('scan_rotator')
        # Use RELIABLE QoS profile to match publisher if needed
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=30
        )
        self.sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_cb, qos_profile)
        self.pub = self.create_publisher(
            LaserScan, '/scan_rotated', qos_profile)
        self.get_logger().info("Scan Rotator Node Started")  # Debug message

    def scan_cb(self, scan: LaserScan):
        # create new message
        out = LaserScan()
        out.header        = scan.header
        out.header.frame_id = 'lidar_rotate_link' # keep the same frame!
        out.angle_increment = scan.angle_increment
        out.time_increment  = scan.time_increment
        out.scan_time       = scan.scan_time
        out.range_min       = scan.range_min
        out.range_max       = scan.range_max

        # rotate the scan window +90°: shift min/max
        theta = math.pi/4
        out.angle_min = scan.angle_min + theta
        out.angle_max = scan.angle_max + theta

        # compute circular shift of ranges
        bins = len(scan.ranges)
        shift = int(theta / scan.angle_increment) % bins
        out.ranges = scan.ranges[-shift:] + scan.ranges[:-shift]

        if scan.intensities:
            out.intensities = scan.intensities[-shift:] + scan.intensities[:-shift]

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRotator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
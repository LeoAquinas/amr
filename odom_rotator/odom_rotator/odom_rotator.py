import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomAxisSwap(Node):
    def __init__(self):
        super().__init__('odom_axis_swap')
        self.sub = self.create_subscription(Odometry, '/rtabmap_odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, '/rtabmap_odom_axis_swapped', 10)
        self.get_logger().info('Odom axis swap node started.')

    def odom_callback(self, msg):
        x_old = msg.pose.pose.position.x
        y_old = msg.pose.pose.position.y

        # Swap x and y. Change sign if needed, e.g., y_new = -x_old if your axes are flipped
        x_new = y_old
        y_new = x_old

        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id

        # Position with swapped axes
        new_msg.pose.pose.position.x = -x_new
        new_msg.pose.pose.position.y = y_new
        new_msg.pose.pose.position.z = msg.pose.pose.position.z

        # Keep orientation unchanged
        new_msg.pose.pose.orientation = msg.pose.pose.orientation

        # Keep twist unchanged
        new_msg.twist = msg.twist

        self.pub.publish(new_msg)
        self.get_logger().info(f'Position swapped: old x={x_old:.3f}, y={y_old:.3f} -> new x={x_new:.3f}, y={y_new:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomAxisSwap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# import math

# def quaternion_multiply(q1, q2):
#     # q = [x, y, z, w]
#     x1, y1, z1, w1 = q1
#     x2, y2, z2, w2 = q2
#     x = w1*x2 + x1*w2 + y1*z2 - z1*y2
#     y = w1*y2 - x1*z2 + y1*w2 + z1*x2
#     z = w1*z2 + x1*y2 - y1*x2 + z1*w2
#     w = w1*w2 - x1*x2 - y1*y2 - z1*z2
#     return [x, y, z, w]

# def quaternion_normalize(q):
#     x, y, z, w = q
#     norm = math.sqrt(x*x + y*y + z*z + w*w)
#     if norm == 0:
#         return [0, 0, 0, 1]  # default to identity quaternion if norm=0
#     return [x / norm, y / norm, z / norm, w / norm]

# def quaternion_from_euler(roll, pitch, yaw):
#     # Convert Euler angles (radians) to quaternion [x, y, z, w]
#     cy = math.cos(yaw * 0.5)
#     sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5)
#     sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5)
#     sr = math.sin(roll * 0.5)

#     w = cr * cp * cy + sr * sp * sy
#     x = sr * cp * cy - cr * sp * sy
#     y = cr * sp * cy + sr * cp * sy
#     z = cr * cp * sy - sr * sp * cy
#     return [x, y, z, w]

# class OdomRotator(Node):
#     def __init__(self):
#         super().__init__('odom_rotator')
#         self.sub = self.create_subscription(Odometry, '/rtabmap_odom', self.odom_callback, 10)
#         self.pub = self.create_publisher(Odometry, '/rtabmap_odom_aligned', 10)

#         # Rotation quaternion for -90 degrees yaw (Z axis)
#         self.q_rot = quaternion_from_euler(0, 0, -math.pi / 2)

#     def odom_callback(self, msg):
#         # Original position
#         x_old = msg.pose.pose.position.x
#         y_old = msg.pose.pose.position.y
#         z_old = msg.pose.pose.position.z

#         # Rotate position: x_new = y_old, y_new = -x_old
#         x_new = y_old
#         y_new = -x_old

#         # Original quaternion
#         q_orig = [
#             msg.pose.pose.orientation.x,
#             msg.pose.pose.orientation.y,
#             msg.pose.pose.orientation.z,
#             msg.pose.pose.orientation.w,
#         ]

#         # Rotate quaternion and normalize
#         q_new = quaternion_multiply(self.q_rot, q_orig)
#         q_new = quaternion_normalize(q_new)

#         # Log original and new position & orientation
#         self.get_logger().info(
#             f"Original position: x={x_old:.3f}, y={y_old:.3f}, z={z_old:.3f} | "
#             f"Rotated position: x={x_new:.3f}, y={y_new:.3f}, z={z_old:.3f}"
#         )
#         self.get_logger().info(
#             f"Original orientation (quat): x={q_orig[0]:.3f}, y={q_orig[1]:.3f}, "
#             f"z={q_orig[2]:.3f}, w={q_orig[3]:.3f} | "
#             f"Rotated orientation (quat): x={q_new[0]:.3f}, y={q_new[1]:.3f}, "
#             f"z={q_new[2]:.3f}, w={q_new[3]:.3f}"
#         )

#         # Create new odometry message
#         new_msg = Odometry()
#         new_msg.header = msg.header
#         new_msg.child_frame_id = msg.child_frame_id

#         new_msg.pose.pose.position.x = x_new
#         new_msg.pose.pose.position.y = y_new
#         new_msg.pose.pose.position.z = z_old

#         new_msg.pose.pose.orientation.x = q_new[0]
#         new_msg.pose.pose.orientation.y = q_new[1]
#         new_msg.pose.pose.orientation.z = q_new[2]
#         new_msg.pose.pose.orientation.w = q_new[3]

#         new_msg.twist = msg.twist

#         self.pub.publish(new_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomRotator()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter')
        
        # Create subscriber for string commands
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.command_callback,
            10)
        
        # Create publisher for cmd_vel
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        # Default values
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.get_logger().info("CmdVel Converter Node Started")

    def command_callback(self, msg):
        # Convert string command to Twist
        twist = Twist()
        
        # Parse the string command
        command = msg.data.lower()
        
        if command == "forward":
            twist.linear.x = 0.5
        elif command == "backward":
            twist.linear.x = -0.5
        elif command == "left":
            twist.angular.z = 1.0
        elif command == "right":
            twist.angular.z = -1.0
        elif command == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().warn(f"Unknown command: {command}")
            return
        
        # Publish the Twist message
        self.publisher.publish(twist)
        self.get_logger().info(f"Published cmd_vel: {command}")

def main():
    rclpy.init()
    node = CmdVelConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
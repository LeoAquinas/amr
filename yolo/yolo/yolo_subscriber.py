#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import InferenceArray

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        # Changed to InferenceArray
        self.sub = self.create_subscription(
            InferenceArray,  # <- Fix message type
            '/camera/inference_data',
            self.callback,
            1
        )
        self.get_logger().info('YoloSubscriber started, listening on /camera/inference_data')

    def callback(self, msg: InferenceArray):  # <- Fix parameter type
        try:
            # Iterate through received detections
            for detection in msg.inference_result:  # Use correct field name
                self.get_logger().info(
                    f" • {detection.class_name} @ "
                    f"top={detection.top}, left={detection.left}, "
                    f"bottom={detection.bottom}, right={detection.right}"
                )
            self.get_logger().info(msg.inference_result)
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

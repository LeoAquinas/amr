import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class CameraInfoSync(Node):
    def __init__(self):
        super().__init__('camera_info_sync')
        
        self.cam_info_1 = None
        self.cam_info_2 = None

        # Load static CameraInfo from bag or hardcoded file (do this once)
        self.cam_info_1 = self.load_static_info('/t265/fisheye1/camera_info')
        self.cam_info_2 = self.load_static_info('/t265/fisheye2/camera_info')

        self.image_sub_1 = self.create_subscription(
            Image,
            '/t265/fisheye1/image_raw',
            self.image_callback_1,
            10)
        
        self.image_sub_2 = self.create_subscription(
            Image,
            '/t265/fisheye2/image_raw',
            self.image_callback_2,
            10)

        self.pub_info_1 = self.create_publisher(CameraInfo, '/t265/fisheye1/camera_info_sync', 10)
        self.pub_info_2 = self.create_publisher(CameraInfo, '/t265/fisheye2/camera_info_sync', 10)

    def load_static_info(self, topic_name):
        # You can replace this with loading from a YAML or wait for one msg from topic
        msg = CameraInfo()
        msg.k = [1.0]*9
        msg.p = [1.0]*12
        msg.width = 848
        msg.height = 800
        msg.distortion_model = 'plumb_bob'
        return msg

    def image_callback_1(self, msg):
        if self.cam_info_1:
            ci = self.cam_info_1
            ci.header.stamp = msg.header.stamp
            ci.header.frame_id = msg.header.frame_id
            self.pub_info_1.publish(ci)

    def image_callback_2(self, msg):
        if self.cam_info_2:
            ci = self.cam_info_2
            ci.header.stamp = msg.header.stamp
            ci.header.frame_id = msg.header.frame_id
            self.pub_info_2.publish(ci)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

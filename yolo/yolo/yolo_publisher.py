#!/usr/env/bin python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
from ultralytics import YOLO
import cv2
import yaml
from yaml.loader import SafeLoader

# Load the YAML file
with open("/home/heisenburg/agv/src/yolo/coco.yaml", mode ='r') as f:
    data = yaml.load(f, Loader = SafeLoader)

class publisherNode(Node):
    #Main Operating Function
    #This is the parent function which nests the process function listen()
    def __init__(self):
        super().__init__("yolo_publisher")
        
        #Create pubilshers
        self.yolo_img_publisher = self.create_publisher(Image, '/camera/inference_result', 1)
        
        self.bridge = cv_bridge.CvBridge()

        # Load a model
        self.model = YOLO("yolo11n.pt")  # pretrained YOLO11n model

        self.camera_subscriber()


    def camera_subscriber(self):
         self.subscription = self.create_subscription(
              Image,
              '/camera/image_raw',
              self.subscription_callback,
              10
         )
         self.subscription

    def subscription_callback(self, img):
        #  print("In callback")
         try:
            # Convert ROS Image message to OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
            
            # # Display the image using OpenCV
            # cv2.imshow("Camera Image", cv_img)
            # cv2.waitKey(1)  # Wait for 1ms and handle window events

            # Load YOLO model (make sure the model is loaded only once, not inside the callback)
            if not hasattr(self, 'model'):
                self.model = YOLO("yolo11n.pt")  # Load the pretrained YOLO11n model

            # Run inference on the source
            #FOR NOW USE DEFAULT BUILT IN CV2 SHOW ARGUMENT 1ST
            results = self.model(cv_img, show=True)  # list of Results objects

            # View results
            for r in results:
                # print(r.boxes)  # print the Boxes object containing the detection bounding boxes
                boxes = r.boxes.cpu().numpy()
                for box in boxes:
                    bb = box.xyxy[0]  # Bounding box coordinates [x1, y1, x2, y2]
                    label = box.cls  # Class ID
                    confidence = box.conf  # Confidence score
                            
                    # Convert coordinates to integers
                    self.bb_top = int(bb[0])
                    self.bb_left = int(bb[1])
                    self.bb_bottom = int(bb[2])
                    self.bb_right = int(bb[3])

                    #Draw bb
                    cv2.rectangle(cv_img, 
                            (self.bb_top, self.bb_left),
                            (self.bb_bottom, self.bb_right),
                            (255, 255, 0), 2
                    )
                    tag = f"Class: {data['names'][label[0]]}, Confidence: {float(confidence):.3f}"
                    cv2.putText(cv_img, tag, (self.bb_top, self.bb_left), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)

            #Image
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'

            #Publish detection
            self.yolo_img_publisher.publish(img_msg)

         except:
              pass


def main(args=None):
    rclpy.init(args=args)
    node = publisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
	
			

if __name__ == '__main__':
	main()
#!/usr/env/bin python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
from ultralytics import YOLO
import cv2
import yaml
from yaml.loader import SafeLoader
from yolo_msgs.msg import InferenceResult, InferenceArray

# Load the YAML file
with open("/home/heisenburg/agv/src/YOLO/yolo/coco.yaml", mode ='r') as f:
    data = yaml.load(f, Loader = SafeLoader)

class publisherNode(Node):
    #Main Operating Function
    #This is the parent function which nests the process function listen()
    def __init__(self):
        super().__init__("yolo_publisher")
        print("helo")
        
        #Create pubilshers
        self.yolo_img_publisher = self.create_publisher(Image, '/camera/inference_img', 10)

        self.yolo_data_publisher = self.create_publisher(InferenceArray, '/camera/inference_data', 1)
        
        self.bridge = cv_bridge.CvBridge()

        # Load a model
        self.model = YOLO("yolo11n.pt")  # pretrained YOLO11n model

        self.camera_subscriber()


    def camera_subscriber(self):
         self.subscription = self.create_subscription(
              Image,
              '/depth_camera/image_raw',
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
            results = self.model(cv_img, show=False)  # list of Results objects

            # Create publishing array
            inference_array = InferenceArray()

            # View results
            for r in results:
                # print(r.boxes)  # print the Boxes object containing the detection bounding boxes
                boxes = r.boxes.cpu().numpy()
                for box in boxes:
                    bb = box.xyxy[0]  # Bounding box coordinates [x1, y1, x2, y2]
                    label = box.cls  # Class ID
                    confidence = box.conf  # Confidence score
                            
                    # Convert coordinates to integers
                    self.bb_top = int(bb[1])
                    self.bb_left = int(bb[0])
                    self.bb_bottom = int(bb[3])
                    self.bb_right = int(bb[2])

                    #Draw bb
                    cv2.rectangle(cv_img, 
                            (self.bb_left, self.bb_top),
                            (self.bb_right, self.bb_bottom),
                            (255, 255, 0), 2
                    )

                    tag = f"Class: {data['names'][label[0]]}, Confidence: {float(confidence):.3f}"
                    cv2.putText(cv_img, tag, (self.bb_left, self.bb_top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                    # print(self.bb_top)
                    # print(self.bb_left)
                    # print(self.bb_bottom)
                    # print(self.bb_right)

                    data_msg = InferenceResult()
                    data_msg.class_name = data['names'][label[0]]
                    data_msg.top = self.bb_top
                    data_msg.left = self.bb_left
                    data_msg.bottom = self.bb_bottom
                    data_msg.right = self.bb_right

                    inference_array.inference_result.append(data_msg)
                    print(inference_array)

            #Image
            display_img = cv2.resize(cv_img, (128, 96))  # Width, Height
            cv2.imshow("YOLO + Static Box", cv_img)
            cv2.waitKey(1)
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'

            #Publish detection
            self.yolo_img_publisher.publish(img_msg)
            self.yolo_data_publisher.publish(inference_array)

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

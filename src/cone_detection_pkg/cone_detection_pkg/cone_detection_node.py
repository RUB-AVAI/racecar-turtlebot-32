import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import numpy as np
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber
from avai_messages.msg import Detection, DetectionArray, DetectionArrayStamped
from pathlib import Path

class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        self.model = YOLO(Path(__file__).parent / "ml_model"/"best_l.pt")
        self.bridge = CvBridge()

        # Subscribers
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # thresholds for box
        self.min_width = 25
        self.min_height = 25
        self.max_width = 5000
        self.max_height = 5000

        self.subscription_toggle_camera = self.create_subscription(
            Bool,
            '/toggle_camera', 
            self.toggle_camera_callback, 
            10
        )
        self.toggle_camera = True

        # Publisher
        self.publisher_detections = self.create_publisher(DetectionArrayStamped, "detections", 10)
        self.publisher_detections_images = self.create_publisher(Image, "detections/images", 10)

        self.get_logger().info("Cone Detection Node has been started.")

    def toggle_camera_callback(self, msg):
        self.get_logger().info(f"Toggle camera: {msg.data}")
        self.toggle_camera = msg.data

    def image_callback(self, color_msg):
        try:
            # Convert ROS image to OpenCV format
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Perform object detection
        results = self.model.predict(color_image_rgb, save=False, show=False)

        # Initialize DetectionArray message
        detection_array = DetectionArray()

        # Draw bounding boxes and labels on the image
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            labels = result.boxes.cls.cpu().numpy()

            for box, conf, label in zip(boxes, confidences, labels):
                x1, y1, x2, y2 = box

                # skip bounding box if too small or large
                box_width = x2 - x1
                box_height = y2 - y1
                #self.get_logger().info(f'{box_width} {box_height}')

                # filter out bounding boxes
                if box_width < self.min_width or box_height < self.min_height or box_width > self.max_width or box_height > self.max_height:
                   continue

                # Create Detection message
                detection = Detection()
                detection.label = int(label)

                # Append detection to DetectionArray
                detection_array.detections.append(detection)

        # Publish DetectionArray message
        detection_array_stamped = DetectionArrayStamped()
        detection_array_stamped.header = color_msg.header
        detection_array_stamped.detectionarray = detection_array
        self.publisher_detections.publish(detection_array_stamped)
        # TODO: HIER IRGENDWAS IST KAPUTT
        # WIR HABEN ES ALS BGR UND GEBEN BEI ANNOTATED_IMAGE_MSG AN DAS ANNOTATED IMAGE MSG ALS RGB8 ENCODING HAT
        # RESIZE KÖNNEN WIR AUCH IM CAMERA NODE MACHEN? 
        # Convert RGB image back to BGR for saving
        save_image = cv2.cvtColor(color_image_rgb, cv2.COLOR_RGB2BGR)
        save_image = cv2.resize(save_image, (640, 480))
        # Convert the annotated image to a ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(save_image, encoding='rgb8')
        annotated_image_msg.header = color_msg.header
        if self.toggle_camera:
            self.publisher_detections_images.publish(annotated_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
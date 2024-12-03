#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        self.get_logger().info("Cone Detection Node has been started.")

        # Load YOLO model
        self.model = YOLO('/workspace/src/cone_detection_pkg/cone_detection_pkg/ml_model/best.pt')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribers for color and depth images with correct arguments
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')

        # Synchronize color and depth topics
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)


    def image_callback(self, color_msg, depth_msg):
        try:
            # Convert ROS images to OpenCV format
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error('Failed to convert images: %s' % str(e))
            return

        # Convert BGR to RGB for matplotlib display
        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Perform object detection
        results = self.model.predict(color_image_rgb, save=False, show=False)

        # Draw bounding boxes and labels on the image
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            labels = result.boxes.cls.cpu().numpy()

            for box, conf, label in zip(boxes, confidences, labels):
                x1, y1, x2, y2 = box
                # Draw bounding box
                cv2.rectangle(color_image_rgb, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                # Add label and confidence score
                cv2.putText(
                    color_image_rgb,
                    f"Class {int(label)}: {conf:.2f}",
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )

        # Display the image using matplotlib
        plt.figure(figsize=(10, 10))
        plt.imshow(color_image_rgb)
        plt.axis("off")
        plt.show()


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

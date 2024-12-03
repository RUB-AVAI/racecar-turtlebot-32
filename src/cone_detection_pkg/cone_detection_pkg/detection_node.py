#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from message_filters import ApproximateTimeSynchronizer, Subscriber

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info("Detection node has been started.")

        # Load YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
        self.model.conf = 0.5  # Confidence threshold
        self.model.cuda()  # Use GPU if available

        self.bridge = CvBridge()

        self.color_sub = Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        
        # Synchronize color and depth topics
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)


    def image_callback(self, color_msg, depth_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error('Failed to convert images: %s' % str(e))
            return

        # Perform object detection
        results = self.model(color_image)
        detections = results.pandas().xyxy[0]

        for _, row in detections.iterrows():
            x_min = int(row['xmin'])
            y_min = int(row['ymin'])
            x_max = int(row['xmax'])
            y_max = int(row['ymax'])
            label = row['name']
            confidence = row['confidence']

            # Calculate center of the bounding box
            x_center = int((x_min + x_max) / 2)
            y_center = int((y_min + y_max) / 2)

            # Ensure the coordinates are within image bounds
            height, width = depth_image.shape
            if x_center >= width or y_center >= height:
                continue

            # Get depth at the center of the bounding box
            depth = depth_image[y_center, x_center]
            depth_in_meters = depth / 1000.0  # Convert from mm to meters

            # Draw bounding box and label on the image
            cv2.rectangle(color_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(color_image, f"{label} {depth_in_meters:.2f}m", (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the image
        # cv2.imshow('Object Detection', color_image)
        # cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
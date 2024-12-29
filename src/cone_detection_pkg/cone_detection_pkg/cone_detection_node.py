import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import numpy as np
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber
from avai_messages.msg import Detection, DetectionArray

class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        self.get_logger().info("Cone Detection Node has been started.")

        self.model = YOLO('/workspace/src/cone_detection_pkg/cone_detection_pkg/ml_model/best.pt')
        self.bridge = CvBridge()

        # Subscribers
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        
        # thresholds for box
        # TODO: specify values
        self.min_box_size = 50
        self.max_box_size = 200

        # Publisher
        self.publisher_detections = self.create_publisher(DetectionArray, "detections", 10)

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

                # filter out bounding boxes
                print(box_width, box_width)
                if box_width < self.min_box_size or box_height < self.min_box_size or box_width > self.max_box_size or box_height > self.max_box_size:
                    continue 

                # Draw bounding box
                cv2.rectangle(color_image_rgb, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)

                # Calculate center of bounding box
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)

                # Check if center coordinates are within the image
                height, width = depth_image.shape
                if 0 <= x_center < width and 0 <= y_center < height:
                    # Retrieve depth value at center point
                    z_value = depth_image[y_center, x_center]

                    # Check if depth value is valid
                    if np.isfinite(z_value) and z_value > 0:
                        # Convert to meters if necessary (assuming depth in millimeters)
                        z_in_meters = z_value * 0.001  # If depth values are in millimeters
                        z_text = f"Z: {z_in_meters:.2f} m"
                    else:
                        z_in_meters = float('nan')
                        z_text = "Z: N/A"
                else:
                    z_in_meters = float('nan')
                    z_text = "Z: Out of bounds"

                # Draw center point on the image
                cv2.circle(color_image_rgb, (x_center, y_center), radius=5, color=(0, 255, 0), thickness=-1)

                # Add depth value to the image
                cv2.putText(
                    color_image_rgb,
                    z_text,
                    (x_center + 10, y_center),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

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

                # Create Detection message
                detection = Detection()
                detection.x_center = float(x_center)
                detection.y_center = float(y_center)
                detection.z_in_meters = float(z_in_meters)
                detection.label = int(label)

                # Append detection to DetectionArray
                detection_array.detections.append(detection)

        # Publish DetectionArray message
        self.publisher_detections.publish(detection_array)

        # Optionally, save the image using cvbridge

        # Convert RGB image back to BGR for saving
        save_image = cv2.cvtColor(color_image_rgb, cv2.COLOR_RGB2BGR)

        # Save the annotated image
        timestamp = color_msg.header.stamp.sec  # Use ROS message timestamp
        output_path = f"/workspace/src/cone_detection_pkg/cone_detection_pkg/image/detections_{timestamp}.png"  # Change the path as needed
        cv2.imwrite(output_path, save_image)

        self.get_logger().info(f"Image saved: {output_path}")


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

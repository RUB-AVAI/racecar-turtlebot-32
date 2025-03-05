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
        self.model = YOLO(Path(__file__).parent / "ml_model"/"best_nano.pt")
        self.bridge = CvBridge()

        # Subscribers
        self.color_sub2 = self.create_subscription(Image, '/camera/realsense2_camera/color/image_raw', self.image_callback, 60)
        # thresholds for box
        self.min_width = 25
        self.min_height = 25
        self.max_width = 5000
        self.max_height = 5000

        self.subscription_toggle_camera = self.create_subscription(
            Bool,
            '/toggle_camera', self.toggle_camera_callback, 10)
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
            # Convert ROS images to OpenCV format
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert images: %s' % str(e))
            return

        color_image = cv2.resize(color_image, (640, 480))
        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Perform object detection
        results = self.model.predict(color_image_rgb, save=False, show=False, device=0)

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

                colors=[(255,0,0), (0,209,134), (0,255,255)]

                # Draw bounding box
                cv2.rectangle(color_image_rgb, (int(x1), int(y1)), (int(x2), int(y2)), colors[int(label)], 2)

                # Calculate center of bounding box
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)

                z_in_meters = 0  # If depth values are in millimeters
                z_text = "dont care"

                # Draw center point on the image
                # cv2.circle(color_image_rgb, (x_center, y_center), radius=5, color=(0, 255, 0), thickness=-1)

                # Add depth value to the image
                # cv2.putText(
                #     color_image_rgb,
                #     z_text,
                #     (x_center + 10, y_center),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5,
                #     colors[int(label)],
                #     2,
                # )

                # # Add label and confidence score
                # cv2.putText(
                #     color_image_rgb,
                #     f"Class {int(label)}: {conf:.2f}",
                #     (int(x1), int(y1) - 10),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5,
                #     colors[int(label)],
                #     2,
                # )

                # Create Detection message
                detection = Detection()
                detection.z_in_meters = float(z_in_meters)
                detection.angle = float(self.get_angle(x_center))
                detection.label = int(label)

                # self.get_logger().info(f"Detection: classID={detection.label}, angle={detection.angle}, distance={detection.z_in_meters}, x_center={x_center}, y_center={y_center}")

                # Append detection to DetectionArray
                detection_array.detections.append(detection)

        # Publish DetectionArray message
        detection_array_stamped = DetectionArrayStamped()
        detection_array_stamped.header = color_msg.header
        detection_array_stamped.detectionarray = detection_array
        self.publisher_detections.publish(detection_array_stamped)

        # Optionally, save the image using cvbridge

        # Convert RGB image back to BGR for saving
        save_image = color_image_rgb#cv2.cvtColor(color_image_rgb, cv2.COLOR_RGB2BGR)
        #save_image = cv2.resize(save_image, (640, 480))
        # Convert the annotated image to a ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(save_image, encoding='rgb8')
        annotated_image_msg.header = color_msg.header
        if self.toggle_camera:
            self.publisher_detections_images.publish(annotated_image_msg)

        # Save the annotated image
        # timestamp = color_msg.header.stamp.sec  # Use ROS message timestamp
        #output_path = f"/workspace/src/cone_detection_pkg/cone_detection_pkg/image/detections_{time.time}.png"  # Change the path as needed
        #self.publisher_detections_images.publish(save_image)
        #cv2.imwrite(output_path, save_image)
        #self.get_logger().info(f"Image saved: {output_path}")

    def get_angle(self, x_center):
        CAMERA_RGB_FOV = 69  # degrees
        CAMERA_RGB_PIXEL_WIDTH = 640  # pixels
        angle = (x_center - (CAMERA_RGB_PIXEL_WIDTH / 2)) * (CAMERA_RGB_FOV / CAMERA_RGB_PIXEL_WIDTH)
        return angle



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

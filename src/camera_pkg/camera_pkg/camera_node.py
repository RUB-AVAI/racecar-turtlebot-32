import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera node has been started.")

        # Initialize the CvBridge object
        self.bridge = CvBridge()

        # Abonniere Kamera-Topics, um an Bilder und Tiefenwerte der Kamera zu kommen
        self.color_sub = self.create_subscription(
            Image, '/color/image_raw', self.color_callback, qos_profile_system_default)
        self.depth_sub = self.create_subscription(
            Image, '/depth/image_rect_raw', self.depth_callback, qos_profile_system_default)

        # Publishe Bilder und Tiefenwerte für Distanzerkennung
        self.color_pub = self.create_publisher(
            Image, '/camera/color/image_raw', qos_profile_system_default)
        self.depth_pub = self.create_publisher(
            Image, '/camera/aligned_depth_to_color/image_raw', qos_profile_system_default)

    def color_callback(self, msg):  
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Konvertiere zurück zu ROS Image 
            processed_color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            processed_color_msg.header = msg.header

            # Publishe verarbeitet Bilder
            self.color_pub.publish(processed_color_msg)
        except Exception as e:
            self.get_logger().error('Failed to process color image: %s' % str(e))

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

            # Konvertiere zurück zu ROS Image 
            processed_depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            processed_depth_msg.header = msg.header

            # Publishe verarbeitete Tiefenwerte
            self.depth_pub.publish(processed_depth_msg)
        except Exception as e:
            self.get_logger().error('Failed to process depth image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
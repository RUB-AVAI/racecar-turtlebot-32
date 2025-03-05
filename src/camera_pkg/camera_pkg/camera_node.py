import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Initialize the CvBridge object
        self.bridge = CvBridge()

        # Abonniere Kamera-Topics, um an Bilder und Tiefenwerte der Kamera zu kommen
        # '/camera/realsense2_camera/color/image_raw'
        self.color_sub = self.create_subscription(
            Image, '/color/image_raw', self.color_callback, 60)
        # '/camera/realsense2_camera/depth/image_rect_raw'
        # Publishe Bilder und Tiefenwerte für Distanzerkennung
        self.color_pub = self.create_publisher(
            CompressedImage, '/camera/color/image_raw', 60)
        self.get_logger().info("Camera node has been started.")

    def color_callback(self, msg):  
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            color_image = cv2.resize(color_image, (640, 480))
            # Konvertiere zurück zu ROS Image 
            processed_color_msg = self.bridge.cv2_to_compressed_imgmsg(color_image)
            processed_color_msg.header = msg.header

            # Publishe verarbeitet Bilder
            self.color_pub.publish(processed_color_msg)
        except Exception as e:
            self.get_logger().error('Failed to process color image: %s' % str(e))

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
import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera node has been started.")

        # Abonnieren der Topics
        self.color_sub = self.create_subscription(Image, '/color/image_raw', self.color_callback, qos_profile_system_default)
        self.depth_sub = self.create_subscription(Image, '/depth/image_rect_raw', self.depth_callback, qos_profile_system_default)

        # Initialisieren des CvBridge-Objekts
        self.bridge = CvBridge()

    def color_callback(self, msg):
        # Verarbeiten der Farbbild-Daten
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Hier kannst du die Farbbild-Daten verarbeiten

    def depth_callback(self, msg):
        # Verarbeiten der Tiefenbild-Daten
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        points = np.zeros((cv_image.shape[0] * cv_image.shape[1], 3), dtype=np.float32)
        for y in range(cv_image.shape[0]):
            for x in range(cv_image.shape[1]):
                depth = cv_image[y, x]
                if depth!= 0:
                    points[y * cv_image.shape[1] + x] = [x, y, depth]
        np.savetxt('camera_pkg/camera_pkg/depth.pcd', points, fmt='%f')

        

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
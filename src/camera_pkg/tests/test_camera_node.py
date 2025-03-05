import time
import unittest
import numpy as np
import sys
import os
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
sys.path.append(os.path.dirname(__file__)+"/../camera_pkg")
from camera_node import CameraNode

class TestCameraNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize rclpy once for all tests.
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Create a test node to subscribe to the outputs.
        self.test_node = rclpy.create_node("test_camera_node")
        self.camera_node = CameraNode()

        self.received_color_msgs = []
        self.received_depth_msgs = []

        # Subscribe to the topics where CameraNode publishes.
        self.color_sub = self.test_node.create_subscription(
            Image,
            "/camera/color/image_raw",
            lambda msg: self.received_color_msgs.append(msg),
            10)
        self.depth_sub = self.test_node.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            lambda msg: self.received_depth_msgs.append(msg),
            10)


        self.bridge = CvBridge()

    def tearDown(self):
        # Destroy nodes after each test.
        self.test_node.destroy_node()
        self.camera_node.destroy_node()

    def test_color_processing(self):
        """Test that a dummy color image is processed and republished."""
        # Create a dummy color image using a NumPy array.
        color_array = np.zeros((480, 640, 3), dtype=np.uint8)
        dummy_color = self.bridge.cv2_to_imgmsg(color_array, encoding="bgr8")
        dummy_color.header.frame_id = "dummy_color_frame"

        # Call the color callback with the valid dummy image.
        self.camera_node.color_callback(dummy_color)

        end_time = time.time() + 1.0
        while time.time() < end_time and not self.received_color_msgs:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            rclpy.spin_once(self.camera_node, timeout_sec=0.1)

        self.assertGreater(len(self.received_color_msgs), 0,
                           "Camera node did not publish a processed color image.")

    def test_depth_processing(self):
        """Test that a dummy depth image is processed and republished."""
        # Create a dummy depth image using a NumPy array.
        depth_array = np.zeros((480, 640), dtype=np.uint16)
        dummy_depth = self.bridge.cv2_to_imgmsg(depth_array, encoding="16UC1")
        dummy_depth.header.frame_id = "dummy_depth_frame"

        # Call the depth callback with the valid dummy depth image.
        self.camera_node.depth_callback(dummy_depth)

        end_time = time.time() + 1.0
        while time.time() < end_time and not self.received_depth_msgs:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            rclpy.spin_once(self.camera_node, timeout_sec=0.1)

        self.assertGreater(len(self.received_depth_msgs), 0,
                           "Camera node did not publish a processed depth image.")

if __name__ == '__main__':
    unittest.main()

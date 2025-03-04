import time
import unittest

import rclpy
from sensor_msgs.msg import Image
from camera_node import CameraNode  # Adjust import if needed

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
        # Instantiate your CameraNode.
        self.camera_node = CameraNode()

        # Lists to store received messages.
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

    def tearDown(self):
        # Destroy nodes after each test.
        self.test_node.destroy_node()
        self.camera_node.destroy_node()

    def test_color_processing(self):
        """Test that a dummy color image is processed and republished."""
        # Create a dummy Image message. In a real test, you might fill in data that
        # cv_bridge can work with (e.g. a valid header and dummy data).
        dummy_color = Image()
        dummy_color.header.frame_id = "dummy_color_frame"
        # Optionally: add fake image data here if needed.

        # Directly call the callback.
        self.camera_node.color_callback(dummy_color)

        # Allow some time for the message to be published.
        end_time = time.time() + 1.0
        while time.time() < end_time and not self.received_color_msgs:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            rclpy.spin_once(self.camera_node, timeout_sec=0.1)

        self.assertGreater(len(self.received_color_msgs), 0,
                           "Camera node did not publish a processed color image.")

    def test_depth_processing(self):
        """Test that a dummy depth image is processed and republished."""
        dummy_depth = Image()
        dummy_depth.header.frame_id = "dummy_depth_frame"
        # Optionally: add fake depth data if needed.

        self.camera_node.depth_callback(dummy_depth)

        end_time = time.time() + 1.0
        while time.time() < end_time and not self.received_depth_msgs:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            rclpy.spin_once(self.camera_node, timeout_sec=0.1)

        self.assertGreater(len(self.received_depth_msgs), 0,
                           "Camera node did not publish a processed depth image.")

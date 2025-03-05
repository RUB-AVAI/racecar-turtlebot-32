import os
import sys
import time
import unittest
import cv_bridge
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
import cv2
from avai_messages.msg import Detection, DetectionArray, DetectionArrayStamped
from cv_bridge import CvBridge
from std_msgs.msg import Float64
sys.path.append(os.path.dirname(__file__)+"/../cone_detection_pkg")
from cone_detection_node import ConeDetectionNode
from sensor_msgs.msg import Image, CompressedImage

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
        self.test_node = rclpy.create_node("test_cone_detection_node")
        self.cone_detection_node = ConeDetectionNode()

        self.received_detections_msgs = []
        self.received_detection_images_msgs = []

        self.detection_sub = self.test_node.create_subscription(
            DetectionArrayStamped,
            "detections",
            lambda msg: self.received_detections_msgs.append(msg),
            10)

        self.detection_images_sub = self.test_node.create_subscription(
            Image,
            "detections/images",
            lambda msg: self.received_detection_images_msgs.append(msg),
            10)

        self.bridge = CvBridge()

    def tearDown(self):
        # Destroy nodes after each test.
        self.test_node.destroy_node()
        self.cone_detection_node.destroy_node()

    def test_image_callback(self):
        # Lese und konvertiere das Testbild
        img_path = os.path.join(os.path.dirname(__file__), "Image.png")
        color_image = cv2.imread(img_path)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.cone_detection_node.image_callback(color_msg)

        end_time = time.time() + 1.0
        while time.time() < end_time and not self.received_detections_msgs:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            rclpy.spin_once(self.cone_detection_node, timeout_sec=0.1)

        self.assertGreater(len(self.received_detections_msgs), 0,
                           " node did not publish a processed color image.")
        print(self.received_detections_msgs)
        output_txt_path = os.path.join(os.path.dirname(__file__), "Dections.txt")
        with open(output_txt_path, 'w') as file:
        # Iterate over each item in the list and write to the file
            if not self.received_detections_msgs:
                file.write("The list is empty.\n")
            else:
                for index, item in enumerate(self.received_detections_msgs, 1):
                    file.write(f"{item}\n")

        for i in self.received_detection_images_msgs:
            save_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            output_path = os.path.join(os.path.dirname(__file__), "Image_output.png")
            cv2.imwrite(output_path, save_image)
            

        
if __name__ == '__main__':
    unittest.main()

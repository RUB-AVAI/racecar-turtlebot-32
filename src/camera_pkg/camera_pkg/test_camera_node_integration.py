import pytest
import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from.camera_node import CameraNode  # Import your CameraNode class


@pytest.fixture
def node():
    rclpy.init()
    node = CameraNode()
    yield node
    node.destroy_node()


@pytest.fixture
def test_node(node):
    test_node = Node("test_node")
    yield test_node
    test_node.destroy_node()


def test_subscription_and_publication(node, test_node):
    # Subscribe to the processed color and depth topics
    color_sub = test_node.create_subscription(Image, "/camera/color/image_raw", lambda msg: None, qos_profile_system_default)
    depth_sub = test_node.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", lambda msg: None, qos_profile_system_default)

    # Create test publisher
    color_pub = test_node.create_publisher(Image, '/color/image_raw', qos_profile_system_default)
    depth_pub = test_node.create_publisher(Image, '/depth/image_rect_raw', qos_profile_system_default)

    # Publish a sample color and depth message to the input topics
    color_msg = Image()
    color_msg.height = 480
    color_msg.width = 640
    color_msg.encoding = "bgr8"
    color_msg.data = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8).tobytes()
    color_pub.publish(color_msg)

    depth_msg = Image()
    depth_msg.height = 480
    depth_msg.width = 640
    depth_msg.encoding = "16UC1"
    depth_msg.data = np.random.randint(0, 65535, size=(480, 640), dtype=np.uint16).tobytes()
    depth_pub.publish(depth_msg)


def pytest_teardown_module(module):
    rclpy.shutdown()
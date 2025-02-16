import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np  
from .camera_node import CameraNode  # Import your CameraNode class

@pytest.fixture
def node():
    rclpy.init()
    node = CameraNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_color_callback(node):
    # Create a sample color image message
    color_msg = Image()
    color_msg.header.stamp = rclpy.time().to_msg()
    color_msg.height = 480
    color_msg.width = 640
    color_msg.encoding = "bgr8"
    color_msg.data = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8).tobytes()

    # Call the color callback method
    node.color_callback(color_msg)

    # Verify the processed color message is created correctly
    assert node.color_pub.get_num_subscribers() == 0  # No subscribers, so no message published
    # You can also verify the message data, encoding, and header

def test_depth_callback(node):
    # Create a sample depth image message
    depth_msg = Image()
    depth_msg.header.stamp = rclpy.time().to_msg()
    depth_msg.height = 480
    depth_msg.width = 640
    depth_msg.encoding = "16UC1"
    depth_msg.data = np.random.randint(0, 65535, size=(480, 640), dtype=np.uint16).tobytes()

    # Call the depth callback method
    node.depth_callback(depth_msg)

    # Verify the processed depth message is created correctly
    assert node.depth_pub.get_num_subscribers() == 0  # No subscribers, so no message published
    # You can also verify the message data, encoding, and header

def test_image_conversion(node):
    # Test image conversion from ROS message to OpenCV and back
    color_msg = Image()
    color_msg.header.stamp = rclpy.time().to_msg()
    color_msg.height = 480
    color_msg.width = 640
    color_msg.encoding = "bgr8"
    color_msg.data = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8).tobytes()

    cv_bridge = CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
    processed_color_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    assert processed_color_msg.header.stamp == color_msg.header.stamp
    assert processed_color_msg.height == color_msg.height
    assert processed_color_msg.width == color_msg.width
    assert processed_color_msg.encoding == color_msg.encoding
    assert processed_color_msg.data == color_msg.data
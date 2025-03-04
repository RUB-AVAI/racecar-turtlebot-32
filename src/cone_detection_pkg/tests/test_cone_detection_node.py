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
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage

# Launch feature node
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    image_processing_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "camera_pkg", "image_processing_node.py")],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{}]
    )
    return (
        launch.LaunchDescription([
            image_processing_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"image_processing": image_processing_node}
    )

class TestImageProcessingLink(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_image_processing_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_image_processing_transmits(self, image_processing, proc_output):
        msgs_rx = []

        pub = self.node.create_publisher(Image, "raw_image", 10)
        sub = self.node.create_subscription(
            CompressedImage,
            "processed_image",
            lambda msg: msgs_rx.append(msg),
            10
        )

        try:
            # Lese und konvertiere das Testbild
            img_path = os.path.join(os.path.dirname(__file__), "Image.png")
            raw_image = cv2.imread(img_path)
            self.assertIsNotNone(raw_image, "Testbild wurde nicht gefunden.")
            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(raw_image)

            # Kurze Wartezeit, damit alle Nodes bereit sind
            time.sleep(2)
            pub.publish(img_msg)

            # Warte bis zu 10 Sekunden, bis eine verarbeitete Nachricht empfangen wird
            end_time = time.time() + 10
            while time.time() < end_time and not msgs_rx:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # Sicherstellen, dass mindestens eine verarbeitete Nachricht empfangen wurde
            self.assertGreaterEqual(len(msgs_rx), 1, "Es wurde keine verarbeitete Nachricht empfangen.")

            # Optionale Prüfung: Umwandlung der komprimierten Nachricht in ein OpenCV-Bild
            processed_cv_img = bridge.compressed_imgmsg_to_cv2(msgs_rx[0])
            self.assertIsNotNone(processed_cv_img, "Die Umwandlung der verarbeiteten Nachricht in ein OpenCV-Bild schlug fehl.")
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)

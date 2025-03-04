import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraTopic(Node):
    def __init__(self):
        super().__init__('camera_topic')

        self.color_pub = self.create_publisher(
            Image, '/color/image_raw', qos_profile_system_default)
        
        self.color_sub = self.create_subscription(
            Image, '/camera/realsense2_camera/color/image_raw', lambda msg: self.color_pub.publish(msg), qos_profile_system_default)
        
        self.get_logger().info("Camera node topic compat has been started.")


def main(args=None):
    rclpy.init(args=args)
    node = CameraTopic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
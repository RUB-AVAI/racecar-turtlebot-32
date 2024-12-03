import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # Assuming cone detection info is published as a String for simplicity

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            String,
            'cone_detection_topic',
            self.cone_detection_callback,
            10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.lidar_data = None

    def cone_detection_callback(self, msg):
        self.get_logger().info(f'Received cone detection data: {msg.data}')
        if self.lidar_data:
            self.process_cone_detection(msg.data)

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def process_cone_detection(self, cone_data):
        # Process the cone detection data and find corresponding LiDAR points
        # This is a simplified example
        self.get_logger().info(f'Processing cone detection data with LiDAR data')

        # Example processing (to be replaced with actual logic)
        for i, range in enumerate(self.lidar_data.ranges):
            angle = self.lidar_data.angle_min + i * self.lidar_data.angle_increment
            self.get_logger().info(f'Angle: {angle}, Range: {range}')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
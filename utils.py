import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumblePublisher(Node):

    def __init__(self):
        super().__init__('humble_publisher')
        self.publisher_ = self.create_publisher(String, 'humble_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Fortni535354te: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    humble_publisher = HumblePublisher()

    rclpy.spin(humble_publisher)

    humble_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
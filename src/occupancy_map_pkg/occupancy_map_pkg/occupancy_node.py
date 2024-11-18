import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OccupancyNode(Node):
    def __init__(self):
        super().__init__('occupancy_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(2.0, self.timer_callback)

    def odom_callback(self, msg):
        self.get_logger().info('Received odometry data')
        # Process the odometry data here
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular

        self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        self.get_logger().info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
        self.get_logger().info(f"Linear Velocity: x={linear_velocity.x}, y={linear_velocity.y}, z={linear_velocity.z}")
        self.get_logger().info(f"Angular Velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")
    
    def timer_callback(self):
        self.get_logger().info('occupancynode running')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
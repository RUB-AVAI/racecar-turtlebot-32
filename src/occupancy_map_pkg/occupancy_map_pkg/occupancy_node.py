import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QPainter, QColor
from PyQt5.QtCore import QTimer

class OccupancyNode(Node):
    def __init__(self):
        super().__init__('occupancy_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.map_size = 500
        self.map = QPixmap(self.map_size, self.map_size)
        self.map.fill(QColor('white'))
        self.position = (self.map_size // 2, self.map_size // 2)

        self.timer = self.create_timer(2.0, self.timer_callback)

    def odom_callback(self, msg):
        self.get_logger().info('Received odometry data')
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular

        self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        self.get_logger().info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
        self.get_logger().info(f"Linear Velocity: x={linear_velocity.x}, y={linear_velocity.y}, z={linear_velocity.z}")
        self.get_logger().info(f"Angular Velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")

        r, p, y = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.get_logger().info(f"Transformed Orientation: r={r}, p={p}, y={y}")

        # Update the position on the map
        self.position = (int(position.x * 10) + self.map_size // 2, int(position.y * 10) + self.map_size // 2)
        self.update_map()

    def update_map(self):
        painter = QPainter(self.map)
        painter.setPen(QColor('black'))
        painter.drawPoint(self.position[0], self.position[1])
        painter.end()

    def timer_callback(self):
        self.get_logger().info('occupancynode running')

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Occupancy Map')
        self.setGeometry(100, 100, node.map_size, node.map_size)
        self.label = QLabel(self)
        self.label.setGeometry(0, 0, node.map_size, node.map_size)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)  # Update every 100 ms

    def update_map(self):
        self.label.setPixmap(self.node.map)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyNode()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
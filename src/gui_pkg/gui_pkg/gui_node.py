import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QPainter, QColor
from PyQt5.QtCore import QTimer
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

class GuiNode(Node):
    hmi = None  # MainWindow

    def __init__(self):
        super().__init__("gui_node")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Occupancy Map')
        self.setGeometry(100, 100, 600, 600)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)  # Update every 100 ms

    def update_map(self):
        pass#self.label.setPixmap(self.node.map)
        self.node.get_logger().info("Updating map...")

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    hmi = MainWindow()

    node = GuiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    hmi.node = node
    node.hmi = hmi

    thread = Thread(target=executor.spin)
    thread.start()

    try:
        hmi.show()
        sys.exit(app.exec_())
    finally:
        node.get_logger().info("Shutting down GUI node...")
        node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
"""
msg
.points
    [[posx,posy,label]]
.botpos
    posx,posy,rotation
    """
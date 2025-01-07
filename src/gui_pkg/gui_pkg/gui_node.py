import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QCheckBox, QDoubleSpinBox, QVBoxLayout, QWidget, QWidget, QHBoxLayout
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor
from PyQt5.QtCore import QTimer, Qt
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from ackermann_msgs.msg import AckermannDriveStamped
from avai_messages.msg import OccupancyMapState
from sensor_msgs.msg import Image
import message_filters
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np
from cv_bridge import CvBridge

class GuiNode(Node):
    hmi = None  # MainWindow
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info("Gui node has been started.")
        self.subscription_occupancymap = message_filters.Subscriber(
            self,
            OccupancyMapState,
            '/occupancy_map')
        self.subscription_camera = message_filters.Subscriber(
            self,
            Image,
            '/camera/color/image_raw')
        self.subscription_occupancymap.registerCallback(self.occupancy_map_callback)
        self.subscription_camera.registerCallback(self.camera_callback)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.classedpoints = None
        self.turtle = None
        self.bridge = CvBridge()  # Initialize CvBridge
        self.image = None  # Initialize image
    def occupancy_map_callback(self, msg):
        self.classedpoints = msg.classedpoints
        self.turtle = msg.turtle
    def camera_callback(self, msg):
        self.get_logger().info(f"MESSAGE: {msg}")
        # Convert the ROS Image message to an OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

class MainWindow(QMainWindow):
    def __init__(self, gui_node):
        super().__init__()
        self.setWindowTitle('Occupancy Map')
        self.setGeometry(100, 100, 900, 600)
        self.gui_node = gui_node # for receiving and publishing messages (occupancymap and drive)
        # Set focus policy to ensure the window receives key press events
        self.setFocusPolicy(Qt.StrongFocus)

        # Create a central widget and set the layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)  # Align everything to the top and left

        # description text
        self.text_label = QLabel('Use the W A S D keys to control the robot', self)
        self.text_label.setGeometry(10, 10, 200, 30)  # Adjust the position and size as needed
        layout.addWidget(self.text_label)

        # checkbox to enable/disable keyPressEvent
        self.checkbox = QCheckBox('Enable Key Press', self)
        self.checkbox.setGeometry(10, 50, 150, 30)
        self.checkbox.setChecked(True)  # Enable by default
        layout.addWidget(self.checkbox)

        # SPEED INPUT
        speed_layout = QHBoxLayout()
        self.speed_label = QLabel('Enter speed value:', self)
        self.speed_label.setGeometry(10, 90, 150, 30)  # Adjust the position and size as needed
        speed_layout.addWidget(self.speed_label)
        self.speed_input = QDoubleSpinBox(self)
        self.speed_input.setGeometry(10, 90, 150, 30)
        self.speed_input.setRange(0.0, 100.0)  # Set the range of acceptable values
        self.speed_input.setDecimals(2)  # Set the number of decimal places
        self.speed_input.setSingleStep(0.1)  # Set the step size
        self.speed_input.setValue(1.0)  # Set the initial value
        speed_layout.addWidget(self.speed_input)
        layout.addLayout(speed_layout)

        # STEERING ANGLE INPUT
        steering_layout = QHBoxLayout()
        self.steering_label = QLabel('Enter steering angle value:', self)
        self.steering_label.setGeometry(10, 90, 150, 30)  # Adjust the position and size as needed
        steering_layout.addWidget(self.steering_label)
        self.steering_input = QDoubleSpinBox(self)
        self.steering_input.setGeometry(10, 90, 150, 30)
        self.steering_input.setRange(0.0, 100.0)  # Set the range of acceptable values
        self.steering_input.setDecimals(2)  # Set the number of decimal places
        self.steering_input.setSingleStep(0.1)  # Set the step size
        self.steering_input.setValue(1.0)  # Set the initial value
        steering_layout.addWidget(self.steering_input)
        layout.addLayout(steering_layout)

        # STEERING ANGLE VELOCITY INPUT
        steering_vel_layout = QHBoxLayout()
        self.steering_vel_label = QLabel('Enter steering angle velocity value:', self)
        self.steering_vel_label.setGeometry(10, 90, 150, 30)  # Adjust the position and size as needed
        steering_vel_layout.addWidget(self.steering_vel_label)
        self.steering_vel_input = QDoubleSpinBox(self)
        self.steering_vel_input.setGeometry(10, 90, 150, 30)
        self.steering_vel_input.setRange(0.0, 100.0)  # Set the range of acceptable values
        self.steering_vel_input.setDecimals(2)  # Set the number of decimal places
        self.steering_vel_input.setSingleStep(0.1)  # Set the step size
        self.steering_vel_input.setValue(0.0)  # Set the initial value
        steering_vel_layout.addWidget(self.steering_vel_input)
        layout.addLayout(steering_vel_layout)

        occupancymap_layout = QHBoxLayout()
        # Create a Matplotlib figure and canvas
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        occupancymap_layout.addWidget(self.canvas)
        # Create a label to display the image
        self.image_label = QLabel(self)
        occupancymap_layout.addWidget(self.image_label)
        layout.addLayout(occupancymap_layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(1000)  # Update every 1000 ms (1 second)


    def update_map(self):
        self.gui_node.get_logger().info("Updating map....")
        # Clear the figure
        self.figure.clear()

        # Create a scatter plot
        ax = self.figure.add_subplot(111)
        x = []
        y = []
        color = []
        if self.gui_node.turtle:
            # Visualize the turtle's position as a blue dot
            x.append(self.gui_node.turtle.x)
            y.append(self.gui_node.turtle.y)
            color.append(3)  # Turtle color
            # Visualize the turtle's angle as a red arrow
            turtle_x = self.gui_node.turtle.x
            turtle_y = self.gui_node.turtle.y
            turtle_angle = self.gui_node.turtle.angle
            ax.quiver(turtle_x, turtle_y, np.cos(turtle_angle), np.sin(turtle_angle), scale=10, color='red')
        if self.gui_node.classedpoints:
            for point in self.gui_node.classedpoints:
                x.append(point.x)
                y.append(point.y)
                color.append(point.c)
        ax.scatter(x, y, c=color, cmap='viridis', vmin=0, vmax=3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Occupancy Map')

        # Refresh the canvas
        self.canvas.draw()    
        # Update the image label
        if self.gui_node.image is not None:
            self.gui_node.get_logger().info("Update IMAGE..")
            # Convert the OpenCV image to QImage
            height, width, channel = self.gui_node.image.shape
            bytes_per_line = 3 * width
            q_image = QImage(self.gui_node.image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(q_image))

    def keyPressEvent(self, event):
        if not self.checkbox.isChecked() or event.key() not in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            print("nothing")
            return  # Do nothing if the checkbox is not checked

        msg = AckermannDriveStamped()
        speed_value = self.speed_input.value()  # Get the entered speed value
        steering_value = self.steering_input.value()  # Get the entered steering angle value
        steering_velocity_value = self.steering_vel_input.value()  # Get the entered steering angle velocity value
        if event.key() == Qt.Key_W:
            msg.drive.speed = speed_value
            msg.drive.steering_angle = 0.0
            msg.drive.steering_angle_velocity = 0.0
            print("W key pressed")
        elif event.key() == Qt.Key_S:
            msg.drive.speed = -speed_value
            msg.drive.steering_angle = 0.0
            msg.drive.steering_angle_velocity = 0.0
            print("S key pressed")
        elif event.key() == Qt.Key_A:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = steering_value
            msg.drive.steering_angle_velocity = steering_velocity_value
            print("A key pressed")
        elif event.key() == Qt.Key_D:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = -steering_value
            msg.drive.steering_angle_velocity = steering_velocity_value
            print("D key pressed")
        
        self.gui_node.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = GuiNode()

    app = QApplication(sys.argv)
    hmi = MainWindow(node)

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
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QCheckBox, QDoubleSpinBox, QVBoxLayout, QWidget, QWidget, QHBoxLayout
from PyQt5.QtGui import QPixmap, QPainter, QColor
from PyQt5.QtCore import QTimer, Qt
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from ackermann_msgs.msg import AckermannDriveStamped

class GuiNode(Node):
    hmi = None  # MainWindow
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info("Gui node has been started.")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)



class MainWindow(QMainWindow):
    def __init__(self, gui_node):
        super().__init__()
        self.setWindowTitle('Occupancy Map')
        self.setGeometry(100, 100, 600, 600)
        self.gui_node = gui_node # for publishing messages
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

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)  # Update every 100 ms

    def update_map(self):
        pass#self.label.setPixmap(self.node.map)
        #self.node.get_logger().info("Updating map...")

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
import rclpy
from rclpy.node import Node
import sys
from nav_msgs.msg import Odometry
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QCheckBox, QDoubleSpinBox, QVBoxLayout, QWidget, QWidget, QHBoxLayout, QPushButton
from rclpy.qos import qos_profile_system_default
from PyQt5.QtGui import QPixmap, QPainter, QColor, QImage
from PyQt5.QtCore import QTimer, Qt
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from ackermann_msgs.msg import AckermannDriveStamped
from avai_messages.msg import OccupancyMapState, Polygon, DetectionArrayStamped
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import message_filters
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np

class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.hmi : "MainWindow"= None  # MainWindow

        self.get_logger().info("Gui node has been started.")
        # Subscribe to the occupancy map and camera topics
        self.subscription_occupancymap = message_filters.Subscriber(
            self,
            OccupancyMapState,
            '/occupancy_map')
        self.subscription_middlepoint = message_filters.Subscriber( # contains middle points of yellow and blue cone
            self,
            Polygon,
            '/middle_point')
        self.subscription_odom = message_filters.Subscriber(
            self,
            Odometry,
            '/odom')
        self.subscription_annotated_image = self.create_subscription(
            Image,
            'detections/images',
            self.annotated_image_callback,
            10)
        self.subscription_lidar_points = self.create_subscription(
            DetectionArrayStamped,
            '/fused_detections',
            self.lidar_points_callback,
            10)
        self.reset_occupancy_map = self.create_publisher(
            Bool, '/reset_occupancy_map', qos_profile_system_default)
        self.toggle_autodrive = self.create_publisher(
            Bool, '/toggle_autdrive', qos_profile_system_default)
        self.toggle_camera = self.create_publisher(
            Bool, '/toggle_camera', qos_profile_system_default)
        self.publisher_middlepoint_width = self.create_publisher(
            Float32, '/middlepoint_width', qos_profile_system_default)

        self.subscription_occupancymap.registerCallback(self.occupancy_map_callback)
        self.subscription_middlepoint.registerCallback(self.middlepoint_callback)
        # self.subscription_odom.registerCallback(self.callback_test)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.image = None
        self.classedpoints = None
        self.lidarpoints = None
        self.turtle = None
        self.middlepoints = []
    # def callback_test(self, msg):
        # self.get_logger().info('Received odometry data')
        # # Process the odometry data here
        # position = msg.pose.pose.position
        # orientation = msg.pose.pose.orientation
        # linear_velocity = msg.twist.twist.linear
        # angular_velocity = msg.twist.twist.angular

        # self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        # self.get_logger().info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
        # self.get_logger().info(f"Linear Velocity: x={linear_velocity.x}, y={linear_velocity.y}, z={linear_velocity.z}")
        # self.get_logger().info(f"Angular Velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")

    def lidar_points_callback(self, msg):
        self.get_logger().info('Received lidar points')
        self.lidarpoints = msg.detectionarray.detections
        self.hmi.update_map()

    def middlepoint_callback(self, msg):
        self.get_logger().info('Received middle points')
        self.middlepoints = msg.points
        self.hmi.update_map()
    def occupancy_map_callback(self, msg):
        self.classedpoints = msg.classedpoints
        self.turtle = msg.turtle
        self.hmi.update_map()
    def annotated_image_callback(self, msg):
        # Convert the ROS Image message to a QImage
        try:
            height = msg.height
            width = msg.width
            channels = 3  # Assuming BGR8 encoding
            bytes_per_line = channels * width
            q_image = QImage(msg.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.hmi.update_image(q_image)
        except Exception as e:
            self.get_logger().error('Failed to convert annotated image: %s' % str(e))

class MainWindow(QMainWindow):
    def __init__(self, gui_node :GuiNode):
        super().__init__()
        self.keyslist = set()
        self.setWindowTitle('Occupancy Map')
        self.setGeometry(100, 100, 600, 600)
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
        self.checkbox.setChecked(False)
        self.checkbox_lidar = QCheckBox('Enable Lidar', self)
        self.checkbox_lidar.setGeometry(10, 50, 150, 30)
        layout.addWidget(self.checkbox)
        layout.addWidget(self.checkbox_lidar)

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

        # X LIMIT INPUTS
        xlim_layout = QHBoxLayout()
        self.xlim_label = QLabel('X Limits:', self)
        xlim_layout.addWidget(self.xlim_label)
        self.xlim_min_input = QDoubleSpinBox(self)
        self.xlim_min_input.setRange(-100.0, 100.0)
        self.xlim_min_input.setDecimals(2)
        self.xlim_min_input.setSingleStep(0.1)
        self.xlim_min_input.setValue(-10.0)
        xlim_layout.addWidget(self.xlim_min_input)
        self.xlim_max_input = QDoubleSpinBox(self)
        self.xlim_max_input.setRange(-100.0, 100.0)
        self.xlim_max_input.setDecimals(2)
        self.xlim_max_input.setSingleStep(0.1)
        self.xlim_max_input.setValue(10.0)
        xlim_layout.addWidget(self.xlim_max_input)
        layout.addLayout(xlim_layout)

        # Y LIMIT INPUTS
        ylim_layout = QHBoxLayout()
        self.ylim_label = QLabel('Y Limits:', self)
        ylim_layout.addWidget(self.ylim_label)
        self.ylim_min_input = QDoubleSpinBox(self)
        self.ylim_min_input.setRange(-100.0, 100.0)
        self.ylim_min_input.setDecimals(2)
        self.ylim_min_input.setSingleStep(0.1)
        self.ylim_min_input.setValue(-10.0)
        ylim_layout.addWidget(self.ylim_min_input)
        self.ylim_max_input = QDoubleSpinBox(self)
        self.ylim_max_input.setRange(-100.0, 100.0)
        self.ylim_max_input.setDecimals(2)
        self.ylim_max_input.setSingleStep(0.1)
        self.ylim_max_input.setValue(10.0)
        ylim_layout.addWidget(self.ylim_max_input)
        layout.addLayout(ylim_layout)

        # AUTO LIMIT CHECKBOX
        self.auto_limit_checkbox = QCheckBox('Auto Limits', self)
        self.auto_limit_checkbox.setChecked(True)
        layout.addWidget(self.auto_limit_checkbox)

        # RESET OCCUPANCY MAP BUTTON
        self.reset_button = QPushButton('Reset Occupancy Map', self)
        self.reset_button.setGeometry(10, 130, 150, 30)
        self.reset_button.clicked.connect(self.reset_occupancy_map)
        layout.addWidget(self.reset_button)

        # TOGGLE AUTODRIVE BUTTON
        self.autodrive_checkbox = QCheckBox('Auto drive', self)
        self.autodrive_checkbox.setChecked(True)
        self.autodrive_checkbox.stateChanged.connect(self.publish_autodrive)
        layout.addWidget(self.autodrive_checkbox)

        # TOGGLE CAMERA BUTTON
        self.camera_checkbox = QCheckBox('Camera', self)
        self.camera_checkbox.setChecked(True)
        self.camera_checkbox.stateChanged.connect(self.publish_toggle_camera)
        layout.addWidget(self.camera_checkbox)

        occupancymap_layout = QHBoxLayout()
        # Create a Matplotlib figure and canvas
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setFixedSize(480, 360)
        occupancymap_layout.addWidget(self.canvas)
        layout.addLayout(occupancymap_layout)

        video_layout = QHBoxLayout()
        # Create a label to display the video
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)
        self.video_label.setScaledContents(True)  # Enable scaled contents
        video_layout.addWidget(self.video_label)
        layout.addLayout(video_layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_video)
        self.timer.start(1000)  # Update every 1000 ms (1 second)

    def reset_occupancy_map(self):
        self.gui_node.get_logger().info("Resetting occupancy map")
        msg = Bool()
        msg.data = True
        self.gui_node.reset_occupancy_map.publish(msg)

    def publish_float(self):
        self.gui_node.get_logger().info("Publishing float value")
        msg = Float32()
        msg.data = self.float_input.value()
        self.gui_node.publisher_middlepoint_width.publish(msg)

    def publish_autodrive(self):
        self.gui_node.get_logger().info("Publishing autodrive state")
        msg = Bool()
        msg.data = self.autodrive_checkbox.isChecked()
        self.gui_node.toggle_autodrive.publish(msg)

    def publish_toggle_camera(self):
        self.gui_node.get_logger().info("Publishing camera state")
        msg = Bool()
        msg.data = self.camera_checkbox.isChecked()
        self.gui_node.toggle_camera.publish(msg)

    def update_image(self, q_image):
        self.video_label.setPixmap(QPixmap.fromImage(q_image))

    def update_video(self):
        if self.gui_node.image is not None:
            self.video_label.setPixmap(QPixmap.fromImage(self.gui_node.image))

    def update_map(self):
        #self.gui_node.get_logger().info("Updating map....")
        # Clear the figure
        self.figure.clear()

        # Create a scatter plot
        ax = self.figure.add_subplot(111)
        x = []
        y = []
        mx = []
        my = []
        color = []
        if not self.checkbox_lidar.isChecked():
            if self.gui_node.classedpoints:
                self.gui_node.lidarpoints = None
                for point in self.gui_node.classedpoints:
                    x.append(point.x)
                    y.append(point.y)
                    if point.c == 0:
                        color.append('yellow')
                    elif point.c == 1:
                        color.append('orange')
                    elif point.c == 2:
                        color.append('blue')
                    else:
                        color.append('green')
                ax.scatter(x, y, c=color)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            #ax.set_xlim((0,15))
            #ax.set_ylim((0,15))

            if not self.auto_limit_checkbox.isChecked():
                ax.set_xlim(self.xlim_min_input.value(), self.xlim_max_input.value())
                ax.set_ylim(self.ylim_min_input.value(), self.ylim_max_input.value())
                        
            if self.gui_node.turtle:
                # Visualize the turtle's position as a blue dot
                x.append(self.gui_node.turtle.x)
                y.append(self.gui_node.turtle.y)
                color.append("red")  # Turtle color
                # Visualize the turtle's angle as a red arrow
                turtle_x = self.gui_node.turtle.x
                turtle_y = self.gui_node.turtle.y
                turtle_angle = self.gui_node.turtle.angle
                ax.quiver(turtle_x, turtle_y, np.cos(turtle_angle), np.sin(turtle_angle), scale=10, color='red')
            if self.gui_node.middlepoints:
                self.gui_node.lidarpoints = None
                for point in self.gui_node.middlepoints:
                    mx.append(point.x)
                    my.append(point.y)
                if mx and my:
                    ax.scatter(mx, my, c='green')
                    ax.plot(mx, my, color='green')  # Connect all points with a line
        elif self.checkbox_lidar.isChecked():
            self.gui_node.classedpoints = None
            self.gui_node.turtle = None
            self.gui_node.middlepoints = None
            # Visualize the lidar points
            for point in self.gui_node.lidarpoints:
                x.append(point.angle)
                y.append(point.z_in_meters)
                if point.label == 0:
                    color.append('yellow')
                elif point.label == 1:
                    color.append('orange')
                elif point.label == 2:
                    color.append('blue')
                elif point.label == 3:
                    color.append('red')
                elif point.label == 4:
                    color.append('purple')
                elif point.label == 5:
                    color.append('cyan')
                elif point.label == 6:
                    color.append('pink')
                elif point.label == 7:
                    color.append('black')
                elif point.label == 8:
                    color.append('brown')
                elif point.label == 9:
                    color.append('magenta')
                elif point.label == 10:
                    color.append('gold')
                elif point.label == 11:
                    color.append('gray')
                else:
                    color.append('green')
            ax.scatter(x, y, c=color)
            ax.set_xlabel('distance')
            ax.set_ylabel('angle')

        #ax.set_title('Occupancy Map')

        # Refresh the canvas
        self.canvas.draw()
    def keyPressEvent(self, event):
        if not self.checkbox.isChecked() or event.key() not in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            print("nothing")
            return  # Do nothing if the checkbox is not checked
        self.keyslist.add(event.key())

        msg = AckermannDriveStamped()
        speed_value = self.speed_input.value()  # Get the entered speed value
        steering_value = self.steering_input.value()  # Get the entered steering angle value
        steering_velocity_value = self.steering_vel_input.value()  # Get the entered steering angle velocity value
        if Qt.Key_W in self.keyslist:
            msg.drive.speed = speed_value
            msg.drive.steering_angle = 0.0
            msg.drive.steering_angle_velocity = 0.0
            print("W key pressed")
        elif Qt.Key_S in self.keyslist:
            msg.drive.speed = -speed_value
            msg.drive.steering_angle = 0.0
            msg.drive.steering_angle_velocity = 0.0
            print("S key pressed")
        if Qt.Key_A in self.keyslist:
            #msg.drive.speed = 0.0
            msg.drive.steering_angle = steering_value
            msg.drive.steering_angle_velocity = steering_velocity_value
            print("A key pressed")
        elif Qt.Key_D in self.keyslist:
            #msg.drive.speed = 0.0
            msg.drive.steering_angle = -steering_value
            msg.drive.steering_angle_velocity = steering_velocity_value
            print("D key pressed")

        self.gui_node.publisher_.publish(msg)
    def keyReleaseEvent(self, event):
        if event.key() in self.keyslist:
            self.keyslist.remove(event.key())

def main(args=None):
    rclpy.init(args=args)

    node = GuiNode()

    app = QApplication(sys.argv)
    hmi = MainWindow(node)
    node.hmi = hmi

    executor = MultiThreadedExecutor()
    executor.add_node(node)

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
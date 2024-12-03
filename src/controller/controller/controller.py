import rclpy
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from geometry_msgs.msg import Twist
from bounding_boxes_msgs.msg import OccupanyMap
import sys
import termios
import tty
from pynput import keyboard
import math

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info("Teleop node has been started.")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subscriber_occupany_map = self.create_subscription(OccupanyMap, 'cones', self.occupancy_callback, qos_profile=qos_profile_system_default)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=False)
        self.listener.start()
    
    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(3)  # Read 3 characters for escape sequences
                if key == '\x03':  # Ctrl+C
                    break
                self.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    def occupancy_callback(self, msg):
        self.get_logger().info("Received cone data")
        points = msg.points
        rob_pos = msg.rob_pos

        if len(points) < 2:
            self.get_logger().info("Not enough points to determine path")
            return

        # Find the two nearest points with different labels
        nearest_points = self.find_nearest_points(points, rob_pos)

        if nearest_points:
            point1, point2 = nearest_points
            midpoint = self.calculate_midpoint(point1, point2)
            self.drive_to_midpoint(midpoint, rob_pos)
        else:
            self.get_logger().info("Could not find two points with different labels")

    def find_nearest_points(self, points, rob_pos):
        min_distance = float('inf')
        nearest_points = None

        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                if points[i].label != points[j].label:
                    distance = self.calculate_distance(points[i], points[j])
                    if distance < min_distance:
                        min_distance = distance
                        nearest_points = (points[i], points[j])

        return nearest_points

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def calculate_midpoint(self, point1, point2):
        midpoint_x = (point1.x + point2.x) / 2
        midpoint_y = (point1.y + point2.y) / 2
        return midpoint_x, midpoint_y

    def drive_to_midpoint(self, midpoint, rob_pos):
        msg = AckermannDriveStamped()
        angle_to_midpoint = math.atan2(midpoint[1] - rob_pos.y, midpoint[0] - rob_pos.x)
        distance_to_midpoint = self.calculate_distance(midpoint, rob_pos)

        msg.drive.speed = min(1.0, distance_to_midpoint)  # Adjust speed based on distance
        msg.drive.steering_angle = angle_to_midpoint

        self.publisher_.publish(msg)

    def on_press(self, key):
        self.get_logger().info("pressed")
        msg = AckermannDriveStamped()
        try:
            if key == keyboard.Key.up:
                msg.drive.speed = 1.0
                msg.drive.steering_angle = 0.0
            elif key == keyboard.Key.down:
                msg.drive.speed = 1.0
                msg.drive.steering_angle = 0.0
            elif key == keyboard.Key.left:
                msg.drive.speed = 1.0
                msg.drive.steering_angle = -0.5
            elif key == keyboard.Key.right:
                msg.drive.speed = 1.0
                msg.drive.steering_angle = 0.5
            self.publisher_.publish(msg)
        except AttributeError:
            pass
    
    def process_key(self, key):
        if key == '\x1b[A':  # Up arrow
            self.on_press(keyboard.Key.up)
        elif key == '\x1b[B':  # Down arrow
            self.on_press(keyboard.Key.down)
        elif key == '\x1b[C':  # Right arrow
            self.on_press(keyboard.Key.right)
        elif key == '\x1b[D':  # Left arrow
            self.on_press(keyboard.Key.left)
        else:
            return

    def on_release(self, key):
        msg = AckermannDriveStamped()
        if key in [keyboard.Key.up, keyboard.Key.down]:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
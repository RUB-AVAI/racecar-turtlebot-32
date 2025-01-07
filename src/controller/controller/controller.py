import rclpy
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from geometry_msgs.msg import Twist
from bounding_boxes_msgs.msg import OccupanyMap
from avai_messages.msg import OccupancyMapState
import sys
import termios
import tty
from math import atan2, copysign, radians, cos, sin, sqrt
from pynput import keyboard
import math

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.target = None
        self.last_target = None
        self.get_logger().info("Teleop node has been started.")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subscriber_occupany_map = self.create_subscription(OccupanyMapState, 'occupancy_map', self.occupancy_callback)
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
        points = msg.classedpoints
        rob_pos = msg.turtle

        if len(points) < 2:
            self.get_logger().info("Not enough points to determine path")
            return
        
        if self.target is not None:
            self.get_logger().info(self.target)
            distance_to_last_target = self.calculate_distance(rob_pos, self.target)
            if distance_to_last_target < 0.1:
                self.last_target = self.target
                self.target = None
        else:
            # Find the two nearest points with different labels
            for step in range(5):
                self.find_nearest_points(points, rob_pos, step)
                self.get_logger().info("Points: " + points)
                if self.target is not None:
                    break

            if self.target:
                point1, point2 = self.target
                self.get_logger().info("Target: " + self.target)
                midpoint = self.calculate_midpoint(point1, point2)
                self.drive_to_midpoint(midpoint, rob_pos)
            else:
                self.get_logger().info("Could not find two points with different labels")

    def find_nearest_points(self, points, rob_pos, line_step):
        distance_threshold = 1
        sign = lambda x: copysign(1, x)
        # get front points
        yellow = []
        blue = []
        # use a 90 degree vector to decide if a point is in front of the bot or not
        # vector goes from bot position to a point (x2,y2) in the direction of the vector
        vect_angle = radians(rob_pos["angle"] + 90)

        line_distance = 0.05
        x1 = cos(rob_pos["angle"]) * line_distance*line_step + rob_pos["x"]
        y1 = sin(rob_pos["angle"]) * line_distance*line_step + rob_pos["y"]
        x2 = cos(vect_angle) * 2 + rob_pos["x"]
        y2 = sin(vect_angle) * 2 + rob_pos["y"]
        for point in points:
            # check if point is "left" or "right" of vector
            front = sign((x2 - x1) * (point[1] - y1) - (y2 - y1) * (point[0] - x1))
            if front < 0:
                distance = sqrt((point[0] - rob_pos["x"]) ** 2 + (point[1] - rob_pos["y"]) ** 2)
                if distance <= distance_threshold:
                    self.get_logger().info(f"conedistance: {distance}")
                    if point[2] == 0:
                        blue.append([point, distance])
                    if point[2] == 2:
                        yellow.append([point, distance])
        if len(yellow) == 0:
            self.get_logger().info("no yellow cones")
            #NEED TO SEARCH HERE
            #self.twist.publish(Twist())
            #self.search(1)  # turn slowly

        elif len(blue) == 0:
            self.get_logger().info("no blue cones")
            #NEED TO SEARCH HERE
            #self.twist.publish(Twist())
            #self.search(0)  # turn slowly in the other direction
            # return
        else:
            yellow.sort(key=lambda x: x[1])
            blue.sort(key=lambda x: x[1])

            min_yellow = yellow[0][0]
            min_blue = blue[0][0]

            middle_point = ((min_yellow[0] - min_blue[0]) * 0.5 + min_blue[0],
                            (min_yellow[1] - min_blue[1]) * 0.5 + min_blue[1])

            self.get_logger().info(str(middle_point))
            if self.target == None:
                self.target = middle_point

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
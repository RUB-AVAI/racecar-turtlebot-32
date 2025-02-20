import rclpy
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from avai_messages.msg import OccupancyMapState,  Polygon, Point
from std_msgs.msg import Bool
import sys
import termios
import tty
from math import atan2, copysign, radians, cos, sin, sqrt
import math
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.target = None
        self.last_target = None
        self.get_logger().info("Teleop node has been started.")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.publish_middlepoints = self.create_publisher(Polygon, '/middle_point', 10)
        self.subscriber_occupany_map = self.create_subscription(OccupancyMapState, 'occupancy_map', self.occupancy_callback, qos_profile_system_default)
        self.subscription_reset_occupancy_map = self.create_subscription(
            Bool,
            '/reset_occupancy_map', self.reset_middlepoints_callback, 10)
        self.subscription_toggle_autodrive = self.create_subscription(
            Bool,
            '/toggle_autodrive', self.drive_reset_steering, 10)
        self.middlepoints = []
        self.toggle_autodrive = True

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

    def reset_middlepoints_callback(self, msg):
        self.get_logger().info("Resetting middlepoints")
        if msg.data:
            self.middlepoints = []

            msg = Polygon()
            msg.points = []
            self.publish_middlepoints.publish(msg)

    def drive_reset_steering(self, msg):
        self.toggle_autodrive = msg.data
        self.get_logger().info(f"Autodrive: {self.toggle_autodrive}")

    def occupancy_callback(self, msg):
        if self.subscription_toggle_autodrive:
            return
        self.get_logger().info("Received cone data")
        points = msg.classedpoints
        rob_pos = msg.turtle

        if len(points) < 2:
            self.get_logger().info("Not enough points to determine path")
            return

        if self.target is not None:
            distance_to_last_target = self.calculate_distance(self.target, rob_pos)
            self.get_logger().info(str(self.target))
            self.get_logger().info(str(distance_to_last_target))
            if distance_to_last_target < 2:
                self.last_target = self.target
                self.target = None
        else:
            # Find the two nearest points with different labels
            for step in range(5):
                self.find_nearest_points(points, rob_pos, step)
                if self.target is not None:
                    break

            if self.target:
                point1, point2 = self.target
                self.get_logger().info("Target: " + str(self.target))
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
        vect_angle = rob_pos.angle + np.pi / 2

        line_distance = 0.05
        x1 = cos(rob_pos.angle) * line_distance*line_step + rob_pos.x
        y1 = sin(rob_pos.angle) * line_distance*line_step + rob_pos.y
        x2 = cos(vect_angle) * 2 + rob_pos.x
        y2 = sin(vect_angle) * 2 + rob_pos.y
        for point in points:
            # check if point is "left" or "right" of vector
            front = sign((x2 - x1) * (point.y - y1) - (y2 - y1) * (point.x - x1))
            if front < 0:
                distance = sqrt((point.x - rob_pos.x) ** 2 + (point.y - rob_pos.y) ** 2)
                if distance <= distance_threshold:
                    self.get_logger().info(f"conedistance: {distance}")
                    if point.c == 2:
                        blue.append([point, distance])
                    if point.c == 0:
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

            middle_point = ((min_yellow.x - min_blue.x) * 0.5 + min_blue.x,
                            (min_yellow.y - min_blue.y) * 0.5 + min_blue.y)

            self.get_logger().info("Middle Point:" + str(middle_point))

            point = Point()
            point.x = middle_point[0]
            point.y = middle_point[1]
            self.middlepoints.append(point)
            points = self.middlepoints

            msg = Polygon()
            msg.points = points
            self.get_logger().info("published middlepoint")
            self.publish_middlepoints.publish(msg)

            if self.target == None:
                self.target = middle_point

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2.x) ** 2 + (point1[1] - point2.y) ** 2)

    def calculate_midpoint(self, point1, point2):
        midpoint_x = (point1 + point2) / 2
        midpoint_y = (point1 + point2) / 2
        return midpoint_x, midpoint_y

    def drive_to_midpoint(self, midpoint, rob_pos):
        msg = AckermannDriveStamped()
        angle_to_midpoint = math.atan2(midpoint[1] - rob_pos.y, midpoint[0] - rob_pos.x)
        distance_to_midpoint = self.calculate_distance(midpoint, rob_pos)
        self.get_logger().info("published middlepoint")
        msg.drive.speed = min(1.0, distance_to_midpoint)  # Adjust speed based on distance
        msg.drive.steering_angle = angle_to_midpoint
        self.get_logger().info(f"Driving to midpoint: {midpoint}, speed: {msg.drive.speed}, angle: {msg.drive.steering_angle}")

        self.publisher_.publish(self)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
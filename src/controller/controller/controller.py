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
from nav_msgs.msg import Odometry
from .tf_transform import euler_from_quaternion

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.target = None
        self.last_target = None
        self.get_logger().info("Teleop node has been started.")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.publish_middlepoints = self.create_publisher(Polygon, '/middle_point', 10)
        self.subscription_odom = self.create_subscription(Odometry,'/odom', self.odom_callback, qos_profile_system_default)
        self.subscriber_occupany_map = self.create_subscription(OccupancyMapState, 'occupancy_map', self.occupancy_callback, qos_profile_system_default)
        self.subscription_reset_occupancy_map = self.create_subscription(
            Bool,
            '/reset_occupancy_map', self.reset_middlepoints_callback, 10)
        self.subscription_toggle_autodrive = self.create_subscription(
            Bool,
            '/toggle_autodrive', self.drive_reset_steering, 10)
        
        self.middlepoints = []
        self.toggle_autodrive = True
        self.points = None
        self.is_stopped = True

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

    
    def check_if_point_is_in_front(self, rob_pos, point):
        """maybe checks the wrong way? idk"""
        vect_angle = rob_pos["angle"] + np.pi / 2
        line_distance = 0.05
        sign = lambda x: copysign(1, x)
        x1 = rob_pos["x"]
        y1 = rob_pos["y"]
        x2 = cos(vect_angle) * 2 + rob_pos["x"]
        y2 = sin(vect_angle) * 2 + rob_pos["y"]

        front = sign((x2 - x1) * (point[1] - y1) - (y2 - y1) * (point[0] - x1))
        return front > 0
    
    def send_drive_msg(self, speed, steering_angle, jerk, acceleration, stop=False):
        """steering range might be [-0.24,0.24] or [-0.5,0.5]"""
        if stop:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            msg.drive.jerk = 0.0
            msg.drive.acceleration = 0.0
            self.publisher_.publish(msg)
            self.is_stopped = True
        elif self.is_stopped:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.6
            msg.drive.steering_angle = steering_angle
            msg.drive.jerk = jerk
            msg.drive.acceleration = acceleration
            self.publisher_.publish(msg)
            self.is_stopped = False
        else:
            msg = AckermannDriveStamped()
            msg.drive.speed = speed
            msg.drive.steering_angle = steering_angle
            msg.drive.jerk = jerk
            msg.drive.acceleration = acceleration
            self.publisher_.publish(msg)

    def odom_callback(self, msg):
        self.get_logger().info("Received odom")
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        r, p, y = euler_from_quaternion([float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)])

        turtle_pos_x = -float(position.x)
        turtle_pos_y = float(position.y)
        turtle_angle = -y
        rob_pos = {
            "x": turtle_pos_x,
            "y": turtle_pos_y,
            "angle": turtle_angle,
        }

        if self.target is not None:
            """distance_to_last_target = self.calculate_distance(self.target, rob_pos)
            #point1, point2 = self.target
            #midpoint = self.calculate_midpoint(point1, point2)
            self.drive_to_midpoint(self.target, rob_pos)
            if distance_to_last_target < 0.25:
                self.last_target = self.target
                self.target = None"""
            self.drive_to_midpoint(self.target, rob_pos)
            if self.check_if_point_is_in_front(rob_pos, self.target):
                self.target = None
                self.send_drive_msg(stop=True)
        else:
            for step in range(5):
                if self.points:
                    self.find_nearest_points(self.points, rob_pos, step)
                    if self.target is not None:
                        self.get_logger().info("found target")
                        break
                    else:
                        self.send_drive_msg(stop=True)



    def occupancy_callback(self, msg):
        #if self.subscription_toggle_autodrive:
        #    return
        self.get_logger().info("Received cone data")
        self.points = msg.classedpoints

    def find_nearest_points(self, points, rob_pos, line_step):
        distance_threshold = 4
        sign = lambda x: copysign(1, x)
        # get front points
        yellow = []
        blue = []
        # use a 90 degree vector to decide if a point is in front of the bot or not
        # vector goes from bot position to a point (x2,y2) in the direction of the vector
        vect_angle = rob_pos["angle"] + np.pi / 2

        line_distance = 0.05
        x1 = cos(rob_pos["angle"]) * line_distance*line_step + rob_pos["x"]
        y1 = sin(rob_pos["angle"]) * line_distance*line_step + rob_pos["y"]
        x2 = cos(vect_angle) * 2 + x1#rob_pos["x"]
        y2 = sin(vect_angle) * 2 + y1#rob_pos["y"]
        for point in points:
            # check if point is "left" or "right" of vector
            front = sign((x2 - x1) * (point.y - y1) - (y2 - y1) * (point.x - x1))
            if front < 0:
                distance = sqrt((point.x - rob_pos["x"]) ** 2 + (point.y - rob_pos["y"]) ** 2)
                if distance <= distance_threshold:
                    self.get_logger().info(f"conedistance: {distance}")
                    if point.c == 2:
                        blue.append([point, distance])
                    if point.c == 0:
                        yellow.append([point, distance])
        """if len(yellow) == 0:
            self.get_logger().info("no yellow cones")
            #NEED TO SEARCH HERE
            #self.twist.publish(Twist())
            #self.search(1)  # turn slowly

        elif len(blue) == 0:
            self.get_logger().info("no blue cones")
            #NEED TO SEARCH HERE
            #self.twist.publish(Twist())
            #self.search(0)  # turn slowly in the other direction
            # return"""
        if len(yellow) > 0 and len(blue) > 0:
            yellow.sort(key=lambda x: x[1])
            blue.sort(key=lambda x: x[1])

            min_yellow = yellow[0][0]
            min_blue = blue[0][0]

            middle_point = ((min_yellow.x + min_blue.x) * 0.5,
                            (min_yellow.y + min_blue.y) * 0.5)
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
        elif len(yellow) == 0 and len(blue) == 0:
            self.send_drive_msg(stop=True)
        elif len(yellow) > 0:
            yellow.sort(key=lambda x: x[1])
            nearest_distance = yellow[0][1]
            if nearest_distance < 1:
                steering = -0.5
            else:
                steering = -0.1
            self.send_drive_msg(0.5,steering, 0.5, 0.5)
        else: # blue > 0
            blue.sort(key=lambda x: x[1])
            nearest_distance = blue[0][1]
            if nearest_distance < 1:
                steering = 0.5
            else:
                steering = 0.1
            self.send_drive_msg(0.5,steering, 0.5, 0.5)

    def calculate_distance(self, midpoint, rob_pos):
        return math.sqrt((midpoint[0] - rob_pos["x"]) ** 2 + (midpoint[1] - rob_pos["y"]) ** 2)

    def calculate_midpoint(self, point1, point2):
        midpoint_x = (point1 + point2) / 2
        midpoint_y = (point1 + point2) / 2
        return midpoint_x, midpoint_y

    def normalize_angle(self, angle):
        angle =  (angle / np.pi)
        return angle

    def rot_from_vec(v: np.ndarray):
        """Calculate the rotation in radian from a vector"""
        rad = np.arctan2(*v[::-1])
        if rad < 0:
            rad += 2 * np.pi
        return rad 

    def drive_to_midpoint(self, midpoint, rob_pos):
        angle_to_midpoint = self.normalize_angle(np.arctan2(midpoint[1] - rob_pos["y"], midpoint[0]- rob_pos["x"]) - rob_pos["angle"])
        distance_to_midpoint = self.calculate_distance(midpoint, rob_pos)
        self.get_logger().info("published middlepoint")
        speed = np.clip(distance_to_midpoint, 0.501, 0.501)
        steering_angle = np.clip(angle_to_midpoint, -0.5, 0.5)
        self.send_drive_msg(speed, steering_angle, jerk=0.5, acceleration=0.5)
        self.get_logger().info(f"Driving to midpoint: {midpoint}, speed: {speed}, angle: {angle_to_midpoint}")


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
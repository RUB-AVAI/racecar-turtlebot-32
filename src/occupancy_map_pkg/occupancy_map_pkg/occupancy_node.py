import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import sys
import numpy as np
from avai_messages.msg import DetectionArrayStamped, OccupancyMapState, ClassedPoint, TurtlebotState
import message_filters

class OccupancyNode(Node):
    def __init__(self):
        super().__init__('occupancy_node')
        self.subscription_odom = message_filters.Subscriber(
            self,
            Odometry,
            '/odom')
        self.subscription_detections = message_filters.Subscriber(
            self,
            DetectionArrayStamped,
            '/detections')
        self.publisher_occupancymap = self.create_publisher(OccupancyMapState, "occupancy_map", 10)
        self.cone_sync = message_filters.ApproximateTimeSynchronizer([self.subscription_odom, self.subscription_detections], 100, .2)
        self.cone_sync.registerCallback(self.callback_synchronised)

        #map is a list of points (x,y,classID)
        self.map = []
        
        self.turtle_pos = [float(0), float(0)]
        self.turtle_angle = float(0)
        self.turtle_state_is_set = False
        
        self.get_logger().info("Occupancy Node has been started.")
        
    def callback_synchronised(self, odom, detections):
        self.odom_callback(odom)
        self.detections_callback(detections)
        self.update_map()
        self.publish_map()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        #self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        #self.get_logger().info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

        r, p, y = euler_from_quaternion([float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)])
        #self.get_logger().info(f"Transformed Orientation: r={r}, p={p}, y={y}")

        self.turtle_pos[0] = float(position.x)
        self.turtle_pos[1] = float(position.y)
        self.turtle_angle = y
        self.turtle_state_is_set = True
        
    def detections_callback(self, msg):
        if(not self.turtle_state_is_set):
            self.get_logger().info("Turtlebot state not set yet")
            return
        
        #self.get_logger().info('Received cones data')
        for detection in msg.detectionarray.detections:
            classID = detection.label
            x = self.turtle_pos[0] + np.cos(detection.angle) * detection.z_in_meters
            y = self.turtle_pos[1] + np.sin(detection.angle) * detection.z_in_meters
            self.map.append((x, y, classID))

    def update_map(self):
        pass

    def publish_map(self):
        points = []
        for point in self.map:
            cp = ClassedPoint()
            cp.x = point[0]
            cp.y = point[1]
            cp.c = point[2]
            points.append(cp)

        turtleState = TurtlebotState()
        turtleState.x = self.turtle_pos[0]
        turtleState.y = self.turtle_pos[1]
        turtleState.angle = self.turtle_angle

        msg = OccupancyMapState(points=points, turtlebotState=turtleState)
        msg.classedpoints = points
        msg.turtle = turtleState
        self.publisher_occupancymap.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
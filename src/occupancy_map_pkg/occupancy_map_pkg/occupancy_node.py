import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from .tf_transform import euler_from_quaternion
from rclpy.qos import qos_profile_system_default
import numpy as np
from avai_messages.msg import DetectionArrayStamped, OccupancyMapState, ClassedPoint, TurtlebotState
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import message_filters
from sklearn.cluster import DBSCAN

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
            'fused_detections') # /detections
        self.cone_sync = message_filters.ApproximateTimeSynchronizer([self.subscription_odom, self.subscription_detections], 1000, .2)
        self.cone_sync.registerCallback(self.callback_synchronised)

        self.publisher_occupancymap = self.create_publisher(OccupancyMapState, "occupancy_map", 10)

        self.subscription_reset_occupancy_map = self.create_subscription(
            Bool,
            '/reset_occupancy_map', self.callback_reset_occupancy_map, qos_profile_system_default)

        # map is a list of points (x, y, classID)
        self.map = []
        self.max_points = 10

        self.turtle_pos = [float(0), float(0)]
        self.turtle_angle = float(0)
        self.turtle_state_is_set = False

        self.get_logger().info("Occupancy Node has been started.")

    def callback_synchronised(self, odom, detections):
        self.odom_callback(odom)
        self.detections_callback(detections)
        self.update_map()
        self.publish_map()

    def callback_reset_occupancy_map(self, msg):
        self.get_logger().info("Resetting occupancy map")
        if msg.data:
            self.map = []
            self.turtle_pos = [float(0), float(0)]
            self.turtle_angle = float(0)
            self.turtle_state_is_set = False

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        #self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        #self.get_logger().info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

        r, p, y = euler_from_quaternion([float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)])
        #self.get_logger().info(f"Transformed Orientation: r={r}, p={p}, y={y}")

        self.turtle_pos[0] = -float(position.x)
        self.turtle_pos[1] = float(position.y)
        self.turtle_angle = -y
        self.turtle_state_is_set = True

    def detections_callback(self, msg):
        if(not self.turtle_state_is_set):
            self.get_logger().info("Turtlebot state not set yet")
            return

        predict_blue_cones = True

        fixed_width = 1

        for detection in msg.detectionarray.detections:
            #if detection.z_in_meters > 0.75:
            #    continue
            classID = detection.label
            angle = -np.deg2rad(detection.angle) + self.turtle_angle
            turtle_angle = self.turtle_angle
            distance = detection.z_in_meters

            if classID == 2 and predict_blue_cones:
                continue

            """if classID == 0 and angle > 0:
                continue

            if classID == 2 and angle < 0:
                continue"""

            x = self.turtle_pos[0] + np.cos(angle) * distance
            y = self.turtle_pos[1] + np.sin(angle) * distance
            #self.get_logger().info(f"Detection: classID={classID}, angle={angle}, turtle_angle={turtle_angle}, distance={distance}, x={x}, y={y}")
            self.map.append((x, y, classID))

            if classID == 0 and predict_blue_cones:
                blue_angle = angle - np.pi / 2
                blue_x = x + np.cos(blue_angle) * fixed_width
                blue_y = y + np.sin(blue_angle) * fixed_width
                self.get_logger().info(f"Predicted Blue Cone: angle={blue_angle}, x={blue_x}, y={blue_y}")
                self.map.append((blue_x, blue_y, 2))

            if len(self.map) > self.max_points:
                self.map = self.map[-self.max_points:]

    def update_map(self):
        points = self.map
        #self.get_logger().info(f"{len(self.map)} points before DBSCAN")
        dbscan = DBSCAN(eps=0.22, min_samples=2)
        if not points:
            return
        labels = dbscan.fit_predict(points)
        clusters = {}
        for i, label in enumerate(labels):
            if label != -1:
                if label not in clusters:
                    clusters[label] = []
                clusters[label].append(points[i])
            else:
                newLabel = 'outlier'
                if newLabel not in clusters:
                    clusters[newLabel] = []
                clusters[newLabel].append(points[i])

        centroids = []
        for label, points in clusters.items():
            if label == 'outlier':
                for point in points:
                    centroids.append(point)
            else:
                points_arr = np.array(points)
                centroid = np.mean(points_arr[:,:2],axis=0)
                centroid = np.append(centroid,points_arr[0,2])
                centroids.append(tuple(centroid))

        self.map = centroids

    def publish_map(self):
        self.get_logger().info(f'Publishing map ({len(self.map)} points)')
        points = []
        for point in self.map:
            cp = ClassedPoint()
            cp.x = point[0]
            cp.y = point[1]
            cp.c = int(point[2])
            points.append(cp)

        turtleState = TurtlebotState()
        turtleState.x = self.turtle_pos[0]
        turtleState.y = self.turtle_pos[1]
        turtleState.angle = self.turtle_angle

        msg = OccupancyMapState()
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
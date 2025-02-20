import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from avai_messages.msg import Detection, DetectionArray, DetectionArrayStamped
from rclpy.qos import qos_profile_sensor_data
import message_filters

def lin_map(x):
    # Maps an index (or angle value) to a normalized value.
    if x < 30:
        return 1 - (1 / 62 * x)
    else:
        return 1 - (1 / 56 * x)

def filter_scan(scan: LaserScan):
    # Optional: set values above a threshold to zero.
    for r in range(len(scan.ranges)):
        if scan.ranges[r] > 1:
            scan.ranges[r] = 0

def cluster(scan: LaserScan):
    # Cluster a region of interest from the scan.
    # For example, use a subset of indices (147 to 203) similar to your original code.
    fov = scan.ranges[147:203]
    index = -1
    error = 0.1
    last = 0
    clusters = []
    for angle, distance in enumerate(fov):
        if distance == 0:
            continue
        # Start a new cluster if difference is large.
        if abs(distance - last) > distance * 0.1:
            clusters.append((angle, angle, distance))
            last = distance
            index += 1
        elif abs(distance - last) <= (distance * error):
            start, end, total = clusters[index]
            clusters[index] = (start, angle, total + distance)
            last = distance
    # Average the distances in each cluster.
    for x in range(len(clusters)):
        start, end, total = clusters[x]
        clusters[x] = (start, end, total / ((end - start) + 1))
    return clusters

class LidarFusion(Node):
    def __init__(self):
        
        super().__init__("lidar_fusion")
        self.lidar_scan = None

        # Subscriber for the lidar scan message.
        self.lidar_sub = message_filters.Subscriber(self, LaserScan, "scan", qos_profile=qos_profile_sensor_data)

        # Subscriber for the YOLO-based detections.
        self.detection_sub = message_filters.Subscriber( self, DetectionArrayStamped, '/detections')

        # Synchronize the two topics.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.detection_sub], queue_size=50, slop=0.1, allow_headerless=False)

        # Publisher for fused detections.
        self.publisher_lidar = self.create_publisher(DetectionArrayStamped, "fused_detections", 10)
        self.get_logger().info("Lidar Fusion Node started (with YOLO detections).")
        self.ts.registerCallback(self.fusion_callback)

    def fusion_callback(self, lidar_scan, detection_msg):
        # Process the lidar scan.
        clusters = cluster(lidar_scan)
        self.get_logger().info(f"Clusters found: {clusters}")

        fused = []  # Will store tuples like (cluster_center, lidar_distance, detection_label)

        # Loop through each detection from the YOLO node.
        # Each detection in detection_msg.detectionarray.detections should include fields: angle, z_in_meters, label.
        for detection in detection_msg.detectionarray.detections:
            for cluster_data in clusters:
                start, end, dist = cluster_data
                # Compute a center value for the cluster.
                cluster_center = (start + end) / 2.0
                # Map the cluster center to a comparable value.
                possible_box = lin_map(cluster_center)
                # If the mapped value is close enough to the detection angle, fuse the data.
                if abs(possible_box - detection.angle) < 0.04:
                    # If multiple clusters match, you might choose the one with the lower distance.
                    fused.append((cluster_center, dist, detection.label))
        
        self.get_logger().info(f"Fused detections: {fused}")

        # Build a DetectionArray message with the fused information.
        fused_msg = DetectionArrayStamped()
        fused_msg.header = detection_msg.header  # Use the detection header timestamp.
        for fused_item in fused:
            cluster_center, lidar_dist, label = fused_item
            det = Detection()
            # You can decide whether to keep the original detection angle or recompute it based on the cluster.
            det.angle = detection.angle  # Using the detection angle from YOLO.
            det.z_in_meters = lidar_dist  # Use the distance computed from lidar.
            det.label = label
            fused_msg.detectionarray.detections.append(det)

        self.publisher_lidar.publish(fused_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

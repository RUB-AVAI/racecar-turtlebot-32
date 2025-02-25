import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from avai_messages.msg import Detection, DetectionArray, DetectionArrayStamped
from rclpy.qos import qos_profile_sensor_data
import message_filters
from sklearn.cluster import DBSCAN
import numpy as np

def normalize_x_center(x_center):
    # all inclusive
    min_x_center = 0
    max_x_center = 275
    min_angle = -34.5
    max_angle = 34.5
    #print(f"x_center: {x_center}, min_x_center: {min_x_center}, max_x_center: {max_x_center}")
    normalized_angle = ((x_center - min_x_center) / (max_x_center - min_x_center)) * (max_angle - min_angle) + min_angle
    #print(f"normalized_angle: {normalized_angle}")
    return normalized_angle


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
            [self.lidar_sub, self.detection_sub], queue_size=300, slop=0.1)

        # Publisher for fused detections.
        self.publisher_lidar = self.create_publisher(DetectionArrayStamped, "fused_detections", 10)
        self.get_logger().info("Lidar Fusion Node started (with YOLO detections).")
        self.ts.registerCallback(self.fusion_callback)

    def fusion_callback(self, lidar_scan, detection_msg):
        # Process the lidar scan.
        clusters = self.cluster(lidar_scan)
        # filter clusters that are too far away
        keepclusters = []
        for cluster in clusters:
            if cluster[0] <= 2:
                keepclusters.append(cluster)
        clusters = np.array(keepclusters)
        if len(clusters) <= 0:
            self.get_logger().info(f"No Clusters")
            return
        # normalize rescale angle values to a range of 69 degrees
        #angles = normalize_x_center(clusters[:,1])
        #self.get_logger().info(f"{angles} {clusters}")
        #clusters = np.stack((clusters[:,0],angles),axis=-1)

        self.get_logger().info(f"Clusters found: {clusters}")
        fused = []  # Will store tuples like (cluster_center, lidar_distance, detection_label)

        # Loop through each detection from the YOLO node.
        # Each detection in detection_msg.detectionarray.detections should include fields: angle, z_in_meters, label.
        for detection in detection_msg.detectionarray.detections:
            for cluster_data in clusters:
                dist, cluster_center = cluster_data
                # Map the cluster center to a comparable value.
                possible_box = normalize_x_center(cluster_center)

                # If the mapped value is close enough to the detection angle, fuse the data.
                self.get_logger().info(f"Possible box: {possible_box}, Detection angle: {detection.angle}, cluster_center: {cluster_center}")
                #self.get_logger().info(f"{possible_box} {detection.angle}")
                if abs(possible_box - detection.angle) < 2:
                    # If multiple clusters match, you might choose the one with the lower distance.
                    fused.append((possible_box, dist, detection.label))

        self.get_logger().info(f"Fused detections: {fused}")

        # Build a DetectionArray message with the fused information.
        fused_msg = DetectionArrayStamped()
        fused_msg.header = detection_msg.header  # Use the detection header timestamp.
        for fused_item in fused:
            cluster_center, lidar_dist, label = fused_item
            det = Detection()
            # You can decide whether to keep the original detection angle or recompute it based on the cluster.
            det.angle = cluster_center # CHANGED IT TO CLUSTER CENTER MAYBE SHOULD BE DETECTION ANGLE (25.02.2025)
            det.z_in_meters = lidar_dist  # Use the distance computed from lidar.
            det.label = label
            fused_msg.detectionarray.detections.append(det)

        #self.publisher_lidar.publish(fused_msg) #NEEDS TO BE ACTIVATED AGAIN


    def cluster(self, scan: LaserScan):
        # Cluster a region of interest from the scan.
        # For example, use a subset of indices (147 to 203) similar to your original code.
        fov_range = 69
        fov_offset = 270//2
        fov = scan.ranges[int((fov_offset-fov_range/2)*4):int((fov_offset+fov_range/2)*4)]
        fused_msg = DetectionArrayStamped()
        for i, distance in enumerate(fov):
                new_angle = normalize_x_center(i)  # Get the real indices
                det = Detection()
                # You can decide whether to keep the original detection angle or recompute it based on the cluster.
                det.angle = float(new_angle) # Using the detection angle from YOLO.
                det.z_in_meters = distance  # Use the distance computed from lidar.
                det.label = 2
                fused_msg.detectionarray.detections.append(det)
        # fov contains 4*69=276 values
        #self.get_logger().info(str([(i,a) for i,a in enumerate(fov) if a < 1.5]))
        indices = np.arange(0, len(fov)).astype(float)
        indices/=100
        fov = np.stack((fov,indices),axis=-1)
        print(fov.shape)
        self.publisher_lidar.publish(fused_msg)
        dbscan = DBSCAN(eps=.0122, min_samples=3)
        labels = dbscan.fit_predict(fov)
        unique_labels = np.unique(labels)
        #self.get_logger().info(f"clusternum: {len(unique_labels)}")
        clusters = []
        for label in unique_labels:
            if label == -1:
                continue
            angles = fov[labels==label]
            if len(angles) < 4 or len(angles) > 50:
                continue
            angles
            # Calculate mean distance
            mean_distance = np.mean(angles[:, 0])

            # Filter out angles that deviate significantly from the mean distance
            threshold = 0.2 * mean_distance  # 10% deviation threshold
            filtered_angles = angles[np.abs(angles[:, 0] - mean_distance) <= threshold]
            #self.get_logger().info(f"{label}: {angles}")
            clusters.append((np.mean(filtered_angles[:,0]), int(filtered_angles[:,1][len(filtered_angles)//2]*100)))
        clusters = np.array(clusters)
        #self.get_logger().info(str(clusters[clusters[:,0]<1]))
        return clusters


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

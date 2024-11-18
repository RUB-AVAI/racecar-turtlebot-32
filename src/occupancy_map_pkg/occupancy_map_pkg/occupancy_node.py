import rclpy

class OccupancyNode(Node):
    def __init__(self):
        super().__init__('occupancy_node')
        self.map = []


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
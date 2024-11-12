import rclpy

class OccupancyNode(Node):
    def __init__(self):
        super().__init__('occupancy_node')


def main(args=None):
    rclpy.init(args=args)


if __name__ == '__main__':
    main()
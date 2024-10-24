import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import termios
import tty

class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/vesc/ackermann_cmd_mux/input/teleop', 10)
        self.get_logger().info('Keyboard controller node has been started.')
        self.run()

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                if key == '\x03':  # Ctrl+C
                    break
                self.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def process_key(self, key):
        msg = AckermannDriveStamped()
        if key == '\x1b[A':  # Up arrow
            msg.drive.speed = 1.0
            msg.drive.steering_angle = 0.0
        elif key == '\x1b[B':  # Down arrow
            msg.drive.speed = -1.0
            msg.drive.steering_angle = 0.0
        elif key == '\x1b[C':  # Right arrow
            msg.drive.speed = 0.5
            msg.drive.steering_angle = -0.5
        elif key == '\x1b[D':  # Left arrow
            msg.drive.speed = 0.5
            msg.drive.steering_angle = 0.5
        else:
            return
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: speed=%f, steering_angle=%f' % (msg.drive.speed, msg.drive.steering_angle))

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
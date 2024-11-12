import rclpy
from rclpy.node import Node
import sys
import termios
from geometry_msgs.msg import Twist
import tty


import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile=qos_profile_system_default)
        self.get_logger().info('Keyboard controller node has been started.')
        self.twist = Twist()
        self.run()

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)  # Read 3 characters for escape sequences
                self.get_logger().info(key)
                if key == '\x03':  # Ctrl+C
                    break
                self.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def process_key(self, key):
        
        if key == 'w':  # Up arrow
            self.twist.linear.x = 1.0
        elif key == 's':  # Down arrow
            self.twist.linear.x = -1.0
        elif key == 'd':  # Right arrow
            self.twist.linear.z = 1.0
        elif key == 'a':  # Left arrow
            self.twist.linear.z = -1.0
        else:
            return
        self.publisher_.publish(self.twist)
        self.get_logger().info('Publishing: linear.x=%f, linear.z=%f' % (self.twist.linear.x, self.twist.linear.z))
       

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
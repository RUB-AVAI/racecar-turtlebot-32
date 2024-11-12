# test.py

import rclpy
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
from pynput import keyboard

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info("Teleop node has been started.")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.twist = Twist()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=False)
        self.listener.start()
    
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

    def on_press(self, key):
        self.get_logger().info("pressed")
        msg = AckermannDriveStamped()
        try:
            if key == keyboard.Key.up:
            	msg.drive.speed = 1.0
            	msg.drive.steering_angle = 0.0
            elif key == keyboard.Key.down:
            	msg.drive.speed = 1.0
            	msg.drive.steering_angle = 0.0
            elif key == keyboard.Key.left:
                msg.drive.speed = 1.0
                msg.drive.steering_angle = -0.5
            elif key == keyboard.Key.right:
            	msg.drive.speed = 1.0
            	msg.drive.steering_angle = 0.5
            self.publisher_.publish(msg)
        except AttributeError:
            pass
    
    def process_key(self, key):
        if key == '\x1b[A':  # Up arrow
            self.on_press(keyboard.Key.up)
        elif key == '\x1b[B':  # Down arrow
            self.on_press(keyboard.Key.down)
        elif key == '\x1b[C':  # Right arrow
            self.on_press(keyboard.Key.right)
        elif key == '\x1b[D':  # Left arrow
            self.on_press(keyboard.Key.left)
        else:
            return

    def on_release(self, key):
    	msg = AckermannDriveStamped()
    	if key in [keyboard.Key.up, keyboard.Key.down]:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
    	elif key in [keyboard.Key.left, keyboard.Key.right]:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
    	self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

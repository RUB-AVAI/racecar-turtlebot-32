# test.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.twist.linear.x = 1.0
            elif key == keyboard.Key.down:
                self.twist.linear.x = -1.0
            elif key == keyboard.Key.left:
                self.twist.angular.z = 1.0
            elif key == keyboard.Key.right:
                self.twist.angular.z = -1.0
            self.publisher_.publish(self.twist)
        except AttributeError:
            pass

    def on_release(self, key):
        if key in [keyboard.Key.up, keyboard.Key.down]:
            self.twist.linear.x = 0.0
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
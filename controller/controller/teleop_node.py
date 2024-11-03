# test.py

import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info("Teleop node has been started.")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile=qos_profile_system_default)
        self.twist = Twist()
        self.create_timer(0.5, lambda: self.on_press(keyboard.Key.up))
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=False)
        self.listener.start()

    def on_press(self, key):
        self.get_logger().info("pressed")
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
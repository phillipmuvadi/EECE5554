#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModifiedListener(Node):
    def __init__(self):
        super().__init__('modified_listener')
        self.sub = self.create_subscription(String, 'chatter', self.callback, 10)

    def callback(self, msg: String):
        # Show received message and its length (a simple "modification" in output)
        self.get_logger().info(f'I heard: "{msg.data}" (len={len(msg.data)})')

def main():
    rclpy.init()
    node = ModifiedListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

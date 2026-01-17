#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def modify_text(s: str) -> str:
    # Example modification: reverse + append
    return s[::-1] + "!"

class ModifiedTalker(Node):
    def __init__(self):
        super().__init__('modified_talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.count = 0

    def on_timer(self):
        msg = String()
        base = f"EECE5554 Lab0 message {self.count}"
        msg.data = modify_text(base)
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = ModifiedTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

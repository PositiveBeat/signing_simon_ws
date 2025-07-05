"""
Servo driver node.
"""

import rclpy
from rclpy.node import Node


class ServoDriverNode(Node):

    def __init__(self):
        super().__init__("servo_driver_node")


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoDriverNode()
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
Servo driver node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ServoDriverNode(Node):

    def __init__(self):
        super().__init__("servo_driver_node")

        # Parameters
        self.declare_parameter("control_frequency", 10.0)

        # Subscriptions
        self.sub_joints_arms = self.create_subscription(
            JointState, "/joint_states", self.callback_joint_states, 1
        )

        # Timers
        self.timer = self.create_timer(
            1.0 / self.control_frequency, self.callback_timer
        )

        # Node variables
        self.servo_commands = {}

    def callback_joint_states(self, msg):
        self.servo_commands = dict(zip(msg.name, msg.position))

    def callback_timer(self):

        if not self.servo_commands:
            self.get_logger().info(f"No commands received yet...", once=True)
        else:
            self.get_logger().info(f"Commands received!", once=True)

        # TODO: Write to servos...


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoDriverNode()
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

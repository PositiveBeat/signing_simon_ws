"""
Servo driver node.
"""

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_driver.src.servo_driver_pca9685 import ServoDriverPCA9685


class ServoDriverNode(Node):

    def __init__(self):
        super().__init__("servo_driver_node")

        # Parameters
        self.declare_parameter("control_frequency", 10.0)
        control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        # Subscriptions
        self.sub_joints_arms = self.create_subscription(
            JointState, "/joint_states", self.callback_joint_states, 1
        )

        # Timers
        self.timer = self.create_timer(1.0 / control_frequency, self.callback_timer)

        # Configure servo manager
        config_folder_path = os.path.join(
            get_package_share_directory("simon_bringup"),
            "config",
        )

        json_files = [
            f"{config_folder_path}/servos_hand.json",
        ]

        self.servo_driver = ServoDriverPCA9685(json_files)
        self.servo_commands = {}

    def callback_joint_states(self, msg):
        self.servo_commands = dict(zip(msg.name, msg.position))

    def callback_timer(self):
        if not self.servo_commands:
            self.get_logger().info(f"No commands received yet...", once=True)
            return
        else:
            self.get_logger().info(f"Commands received!", once=True)

        self.servo_driver.command_servos(self.servo_commands)


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoDriverNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

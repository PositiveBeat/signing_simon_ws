"""
Servo driver node.
"""

import json
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_driver.src.servo_driver_pca9685 import ServoDriverPCA9685


supported_drivers = {"PCA9685": ServoDriverPCA9685}


class ServoDriverNode(Node):

    def __init__(self):
        super().__init__("servo_driver_node")

        # Parameters
        self.declare_parameter("control_frequency", 10.0)
        control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        # Timers
        self.timer = self.create_timer(1.0 / control_frequency, self.callback_timer)

        # Load config files
        config_folder_path = os.path.join(
            get_package_share_directory("simon_bringup"),
            "config",
        )

        json_files = [
            f"{config_folder_path}/servos_arm.json",
            f"{config_folder_path}/servos_hand.json",
        ]

        # Create subscriptions and drivers for all joint groups
        self.subs_joint_states = []
        self.servo_drivers = []
        for json_file in json_files:

            with open(json_file, "r") as file:
                group_config = json.load(file)
                topic_name = group_config["group"]["topic_name"]
                driver_name = group_config["group"]["driver_name"]

                # Subscriptions
                sub = self.create_subscription(
                    JointState, topic_name, self.callback_joint_states, 1
                )
                self.subs_joint_states.append(sub)

                # Drivers
                DriverClass = supported_drivers[driver_name]
                driver = DriverClass(json_files)
                self.servo_drivers.append(driver)

        # Member variables
        self.servo_commands = {}

    def callback_joint_states(self, msg):
        self.servo_commands.update(dict(zip(msg.name, np.rad2deg(msg.position))))

    def callback_timer(self):
        if not self.servo_commands:
            self.get_logger().info(f"No commands received yet...", once=True)
            return
        else:
            self.get_logger().info(f"Commands received!", once=True)

        for servo_driver in self.servo_drivers:
            servo_driver.command_servos(self.servo_commands)

        self.servo_commands = {}


def main(args=None):
    rclpy.init(args=args)

    node_handle = ServoDriverNode()
    rclpy.spin(node_handle)
    node_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

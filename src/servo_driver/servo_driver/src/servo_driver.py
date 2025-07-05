"""
Base class for servo driver/managers.
"""

from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import logging
import numpy as np
from typing import List

from servo_driver.src.servo_control import ServoControl
from .utils import interval_map


@dataclass
class ServoGroup:
    servo_names: List[str]


class ServoDriver(ABC):
    """
    Base class for servo driver/managers.
    This class provides a framework for managing and controlling multiple servos.
    Subclasses must implement abstract methods to handle specific driver functionality.
    """

    def __init__(self, config_files):
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.servo_groups = {}
        self.servos = {}

        # Load and process JSON files
        for json_file in config_files:

            with open(json_file, "r") as file:
                servo_config = json.load(file)

                # Add all servos to flat dict
                self.servos.update(self._add_servos(servo_config["servos"]))

                # Create a ServoGroup object
                group_name = servo_config["group"]["name"]
                servo_names = list(servo_config["servos"].keys())

                self.servo_groups[group_name] = ServoGroup(servo_names=servo_names)

        # Info print
        self.logger.info(f"--------")
        for group_name, group in self.servo_groups.items():
            self.logger.info(f"Added group '{group_name}', containing servos:")

            for servo_name in group.servo_names:
                self.logger.info(f"{servo_name}")
        self.logger.info(f"--------")

        # Setup driver
        self.driver_object = self.setup_driver()
        self.coms_active = self.driver_object is not None

    def _add_servos(self, servo_config):
        """
        Add servos to the driver based on the provided configuration.

        Args:
            servo_config (dict): Dictionary containing servo parameters.
        """

        servo_dict = {}

        for name, parameters in servo_config.items():
            servo_dict[name] = ServoControl(
                servo_name=name,
                servo_id=parameters["servo_id"],
                dir=parameters["dir"],
                gear_ratio=parameters["gear_ratio"],
                pwm_min=parameters["pwm_min"],
                pwm_max=parameters["pwm_max"],
                angle_min=parameters["angle_min"],
                angle_max=parameters["angle_max"],
                angle_software_min=parameters["angle_software_min"],
                angle_software_max=parameters["angle_software_max"],
                default_position=parameters["default_position"],
            )

            self.logger.debug(f"Added servo: {name}")

        return servo_dict

    @abstractmethod
    def setup_driver(self):
        """
        Abstract method to initialize the driver object for communication with servos.
        Must be implemented by subclasses, and return a driver_object.
        """
        pass

    @abstractmethod
    def write_command(self, servo_id, pwm):
        """
        Abstract method to send a command to a servo.

        Args:
            servo_id (int): Servo ID.
            pwm (float): PWM value to command the servo.
        """
        pass

    def command_servos(self, command_dict: dict):
        """
        Send commands to multiple servos using a dictionary of servo names and commands.

        Args:
            command_dict (dict): Dictionary mapping servo names to desired commands.
        """
        if not self.coms_active:
            return

        if not command_dict:
            return

        with ThreadPoolExecutor() as executor:
            futures = [
                executor.submit(self._command_servo, name, command_dict[name])
                for name in self.servos.keys()
                if name in command_dict
            ]
            for future in futures:
                try:
                    future.result()
                except Exception as e:
                    self.logger.error(f"Error commanding servo: {e}")

    def get_default_servo_commands(self):
        """
        Generate a dictionary of default commands for all servos.

        Returns:
            dict: A dictionary mapping servo names to default command values (e.g., 0.0).
        """
        return {name: 0.0 for name in self.servos}

    def get_servo_angles(self):
        """
        Generate a dictionary of current angles for all servos.

        Returns:
            dict: A dictionary mapping servo names to current angles.
        """
        return {name: float(self.servos[name].angle) for name in self.servos}

    def _command_servo(self, name, command):
        """
        Send a command to a single servo.

        Args:
            name (str): Name of the servo.
            command (float): Desired command value (e.g., target angle).
        """
        if np.isnan(command):
            return

        servo = self.servos[name]
        angle_target = command + servo.default_position

        angle_cmd, pwm_cmd = servo.compute_command(angle_target)

        if not self._validate_command(servo, pwm_cmd):
            return

        self.write_command(servo.servo_id, pwm_cmd)

    def _validate_command(self, servo: ServoControl, pwm):
        """
        Validate a command before sending it to a servo.

        Args:
            servo (ServoControl): Servo object being validated.
            pwm (float): PWM value to validate.

        Returns:
            bool: True if the command is valid, False otherwise.
        """
        if not self.coms_active:
            return False

        # Validate PWM
        if (
            pwm is None
            or not isinstance(pwm, (int, float))
            or not (servo.pwm_min <= pwm <= servo.pwm_max)
        ):
            self.logger.warning(f"Invalid PWM: {pwm}")
            return False

        # Validate angle
        angle = servo.pwm_2_angle(pwm)
        angle = servo.gearing_in(angle, servo.gear_ratio)

        # Flip angle if direction is flipped
        if servo.dir < 0:
            angle = interval_map(
                angle,
                servo.angle_software_min,
                servo.angle_software_max,
                servo.angle_software_max,
                servo.angle_software_min,
            )

        if angle < servo.angle_software_min or angle > servo.angle_software_max:
            self.logger.debug(
                f"Servo {servo.servo_id} - PWM {pwm} would result in angle {angle}, "
                f"which is outside the software limits ({servo.angle_software_min}, {servo.angle_software_max})"
            )
            return False

        return True

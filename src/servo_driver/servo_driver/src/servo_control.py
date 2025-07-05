"""
Class for storing attributes and managing limits of a single servo.
"""

import logging
import numpy as np

from .utils import interval_map


class ServoControl:
    """
    Represents a servo motor and enforces limits. It also accounts for gear ratios and direction configurations.
    """

    def __init__(
        self,
        servo_name,
        servo_id,
        pwm_min,
        pwm_max,
        angle_min,
        angle_software_min,
        angle_max,
        angle_software_max,
        default_position=180,
        dir=1,
        gear_ratio=1,
    ):
        """
        Initializes the servo with given parameters.

        Args:
            servo_name (str): Unique name for the servo.
            servo_id (int): Unique identifier for the servo.
            pwm_min (int): Minimum PWM value.
            pwm_max (int): Maximum PWM value.
            angle_min (float): Minimum physical angle of the servo.
            angle_software_min (float): Minimum software-limited angle.
            angle_max (float): Maximum physical angle of the servo.
            angle_software_max (float): Maximum software-limited angle.
            default_position (float, optional): Default angle position. Defaults to 180.
            dir (int, optional): Direction configuration for upside-down placement (1 or -1). Defaults to 1.
            gear_ratio (float, optional): Gear ratio for linked mechanisms. Defaults to 1.
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.servo_name = servo_name
        self.servo_id = servo_id
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.angle_min = angle_min
        self.angle_software_min = angle_software_min
        self.angle_max = angle_max
        self.angle_software_max = angle_software_max
        self.default_position = default_position
        self.dir = dir
        self.gear_ratio = gear_ratio

        self.gear_zero_position = (angle_max + angle_min) // 2
        self.angle_init = self.default_position
        self.angle = self.angle_init
        self.pwm = self.angle_2_pwm(self.angle)

    def compute_command(self, angle_target):
        """
        Computes the PWM command for the servo based on the desired angle.

        Args:
            angle (float): Target angle.
            speed_max (float, optional): Maximum allowed speed. Defaults to None.

        Returns:
            tuple: (angle_cmd, pwm_cmd) as integers.
        """
        angle_cmd = np.clip(
            angle_target, self.angle_software_min, self.angle_software_max
        )

        self.angle = angle_cmd
        self.pwm = self.angle_2_pwm(angle_cmd)

        # Flip angle to send if direction is flipped
        if self.dir < 0:
            angle_cmd = interval_map(
                angle_cmd,
                self.angle_software_min,
                self.angle_software_max,
                self.angle_software_max,
                self.angle_software_min,
            )

        # Apply gearing ratio
        angle_cmd_geared = self.gearing_out(angle_cmd, self.gear_ratio)
        pwm_cmd_geared = self.angle_2_pwm(angle_cmd_geared)

        return int(angle_cmd_geared), int(pwm_cmd_geared)

    def angle_2_pwm(self, angle):
        """
        Converts an angle to a corresponding PWM value.

        Args:
            angle (float): Servo angle.

        Returns:
            int: Corresponding PWM value.
        """
        return int(
            interval_map(
                angle, self.angle_min, self.angle_max, self.pwm_min, self.pwm_max
            )
        )

    def pwm_2_angle(self, pwm):
        """
        Converts a PWM value to its corresponding angle.

        Args:
            pwm (int): PWM signal.

        Returns:
            int: Corresponding angle.
        """
        return int(
            interval_map(
                pwm, self.pwm_min, self.pwm_max, self.angle_min, self.angle_max
            )
        )

    def gearing_in(self, value, gear_ratio):
        """
        Computes the input angle of a gear based on the output angle.

        Args:
            value (float): Output angle.
            gear_ratio (float): Gear ratio.

        Returns:
            float: Input angle.
        """
        return (value - self.gear_zero_position) / gear_ratio + self.gear_zero_position

    def gearing_out(self, value, gear_ratio):
        """
        Computes the output angle of a gear based on the input angle.

        Args:
            value (float): Input angle.
            gear_ratio (float): Gear ratio.

        Returns:
            float: Output angle.
        """
        return self.gear_zero_position + (value - self.gear_zero_position) * gear_ratio

    def reset_position(self, t_d):
        """
        Resets the servo to its initial position.

        Args:
            t_d (float): Time step.

        Returns:
            tuple: (angle_cmd, pwm_cmd)
        """
        return self.reach_angle(t_d, self.angle_init)

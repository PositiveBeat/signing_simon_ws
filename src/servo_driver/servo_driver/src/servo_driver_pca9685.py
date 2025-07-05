"""
Servo driver/manager for servos controlled by a PCA9685.
"""

from adafruit_pca9685 import PCA9685
import board

from servo_driver.src.servo_driver import ServoDriver


class ServoDriverPCA9685(ServoDriver):

    def setup_driver(self):
        self.logger.info("Initializing I2C communication with PCA9685...")

        try:
            i2c = board.I2C()
            pca = PCA9685(i2c)
            pca.frequency = 50

            self.logger.info("I2C communication succesful")
            return pca

        except Exception as e:
            self.logger.error(f"Failed to open port: {e}")
            return None

    def write_command(self, servo_id, pwm):

        self.driver_object.channels[servo_id].duty_cycle = pwm

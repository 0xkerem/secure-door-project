# actuators/servo_control.py

import pigpio
import time
from utils.logger import get_logger

logger = get_logger(__name__)

class ServoController:
    """
    Controls a servo motor using pigpio for accurate PWM.
    """

    def __init__(self, pin: int, move_delay: float = 0.5):
        """
        Initializes pigpio connection and sets GPIO pin for servo.
        """
        self.pin = pin
        self.move_delay = move_delay
        self.pi = pigpio.pi()

        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running. Please start with 'sudo pigpiod'.")

        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        logger.info(f"Servo initialized on GPIO pin {self.pin} using pigpio")

    def set_angle(self, angle: float):
        """
        Moves the servo to a specified angle (0-180 degrees).
        """
        try:
            if not 0 <= angle <= 180:
                raise ValueError("Angle must be between 0 and 180 degrees.")

            # Convert angle to pulse width (500µs - 2500µs typical)
            pulse_width = 500 + (angle / 180.0) * 2000  # µs
            self.pi.set_servo_pulsewidth(self.pin, pulse_width)
            time.sleep(self.move_delay)
            self.pi.set_servo_pulsewidth(self.pin, 0)  # Stop signal to reduce jitter
            logger.info(f"Servo moved to {angle} degrees on GPIO pin {self.pin}")
        except Exception as e:
            logger.error(f"Failed to move servo to {angle}°: {e}")
            raise

    def cleanup(self):
        """
        Stops servo signal and releases GPIO pin.
        """
        try:
            self.pi.set_servo_pulsewidth(self.pin, 0)
            self.pi.stop()
            logger.info(f"Cleaned up GPIO pin {self.pin} and stopped pigpio connection")
        except Exception as e:
            logger.error(f"Failed to cleanup pigpio: {e}")
            raise

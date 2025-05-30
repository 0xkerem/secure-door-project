import RPi.GPIO as GPIO
import time
from utils.logger import get_logger

logger = get_logger(__name__)

class ServoController:
    """
    Controls a servo motor via Raspberry Pi GPIO using PWM signal.
 
 
    Attributes:
        pin (int): BCM GPIO pin number connected to servo signal wire.
        min_duty (float): PWM duty cycle corresponding to 0 degrees.
        max_duty (float): PWM duty cycle corresponding to 180 degrees.
        move_delay (float): Time in seconds to wait for servo movement.
    """

    def __init__(self, pin: int, min_duty: float = 2.0, max_duty: float = 12.0, move_delay: float = 0.5):
        """
        Initializes GPIO pin and PWM for servo control.


        Args:
            pin (int): BCM GPIO pin number.
            min_duty (float): Duty cycle for 0 degrees angle.
            max_duty (float): Duty cycle for 180 degrees angle.
            move_delay (float): Delay after setting servo angle.
        """
        self.pin = pin
        self.min_duty = min_duty
        self.max_duty = max_duty
        self.move_delay = move_delay

        try:
            GPIO.setmode(GPIO.BCM)
            # Check if pin is already configured and log warning if so
            current_function = GPIO.gpio_function(self.pin)
            if current_function in [GPIO.OUT, GPIO.IN]:
                logger.warning(f"GPIO pin {self.pin} may already be configured as {current_function}")

            GPIO.setup(self.pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.pin, 50)  # 50Hz PWM frequency
            self.pwm.start(self.min_duty)  # Initialize PWM with min duty cycle
            logger.info(f"Servo initialized on GPIO pin {self.pin}")
        except Exception as e:
            logger.error(f"Error initializing servo on GPIO pin {self.pin}: {e}")
            raise

    def set_angle(self, angle: float):
        """
        Moves servo motor to the specified angle.

        Args:
            angle (float): Desired servo angle in degrees (0-180).

        Raises:
            ValueError: If angle is outside 0-180 range.
        """
        try:
            if not 0 <= angle <= 180:
                raise ValueError("Angle must be between 0 and 180 degrees.")

            duty_cycle = self.min_duty + (angle / 180.0) * (self.max_duty - self.min_duty)
            self.pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(self.move_delay)
            self.pwm.ChangeDutyCycle(0)  # Stop PWM signal to prevent jitter
            logger.info(f"Servo moved to {angle} degrees on GPIO pin {self.pin}")
        except Exception as e:
            logger.error(f"Failed to move servo to {angle} degrees on GPIO pin {self.pin}: {e}")
            raise

    def cleanup(self):
        """
        Stops PWM and cleans up GPIO pin used for servo.
        """
        try:
            self.pwm.stop()
            GPIO.cleanup(self.pin)
            logger.info(f"Cleaned up GPIO pin {self.pin} for servo")
        except Exception as e:
            logger.error(f"Failed to cleanup GPIO pin {self.pin}: {e}")
            raise

# Note:
# BCM GPIO pin used for servo: define explicitly in project documentation, e.g. GPIO17.

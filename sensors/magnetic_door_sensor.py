# sensors/magnetic_door_sensor.py

import pigpio
from utils.gpio_pins import MAGNETIC_DOOR_SENSOR_PIN
import time

MAGNETIC_SENSOR_PIN = MAGNETIC_DOOR_SENSOR_PIN

class MagneticSensor:
    def __init__(self, pi, pin=MAGNETIC_SENSOR_PIN):
        """
        Initialize the magnetic sensor on the specified GPIO pin using pigpio.
        Args:
            pi (pigpio.pi): The pigpio instance to use.
            pin (int): GPIO pin number to which the magnetic sensor is connected (BCM numbering).
        """
        self.pi = pi
        self.pin = pin
        self.setup()

    def setup(self):
        try:
            self.pi.set_mode(self.pin, pigpio.INPUT)
            self.pi.set_pull_up_down(self.pin, pigpio.PUD_UP)
            print(f"Magnetic sensor setup on GPIO pin {self.pin}")
        except Exception as e:
            print(f"Error setting up magnetic sensor: {e}")

    def read(self):
        """
        Reads the state of the magnetic door sensor.
        Returns:
            int: 1 if the door is closed, 0 if the door is open.
        """
        try:
            return self.pi.read(self.pin)
        except Exception as e:
            print(f"Error reading magnetic sensor: {e}")
            return None

    def calibrate(self, sample_time=5):
        """
        Calibrates the magnetic sensor by reading the sensor state over a period of time.
        Args:
            sample_time (int): Time in seconds to sample the sensor.
        Returns:
            float: Average sensor reading during calibration.
        """
        print(f"Calibrating magnetic sensor for {sample_time} seconds...")
        start_time = time.time()
        readings = []
        while time.time() - start_time < sample_time:
            readings.append(self.read())
            time.sleep(0.1)
        average_reading = sum(readings) / len(readings)
        print(f"Magnetic sensor calibration complete. Average reading: {average_reading}")
        return average_reading

    @staticmethod
    def calculate_totband(value, threshold):
        """
        Calculates the totband value.
        Args:
            value (float): Current sensor reading.
            threshold (float): Threshold value for the totband.
        Returns:
            float: 0 if the value is within the totband, otherwise the value.
        """
        if abs(value) < threshold:
            return 0
        else:
            return value

def magnetic_sensor_main(pi, pin=MAGNETIC_SENSOR_PIN, sample_time=5, threshold=0.1):
    """
    Main function to read and process magnetic door sensor data, including calibration and totband calculation.
    Args:
        pi (pigpio.pi): Instance of pigpio.pi.
        pin (int): GPIO pin number to which the magnetic sensor is connected.
        sample_time (int): Time in seconds to sample the sensor for calibration.
        threshold (float): Threshold value for the totband calculation.
    Returns:
        None
    """
    sensor = MagneticSensor(pi, pin)
    try:
        average_reading = sensor.calibrate(sample_time)
        print("Monitoring magnetic sensor...")
        while True:
            door_state = sensor.read()
            if door_state is not None:
                totband_value = MagneticSensor.calculate_totband(door_state - average_reading, threshold)
                if totband_value == 0:
                    print("Door is closed.")
                else:
                    print("Door is open!")
            else:
                print("Could not read door state.")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting magnetic sensor monitoring.")
    # No explicit pin cleanup needed for pigpio, as pi.stop() in main will handle it

if __name__ == "__main__":
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Could not connect to pigpio daemon! Is pigpiod running?")
    try:
        magnetic_sensor_main(pi)
    except RuntimeError as e:
        print(e)
    finally:
        pi.stop()
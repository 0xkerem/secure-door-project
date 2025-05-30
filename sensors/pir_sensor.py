import RPi.GPIO as GPIO
import time
from utils import filters

# Define GPIO pin for PIR sensor
PIR_PIN = 7  # Example: GPIO 4 (BCM numbering)


def setup_pir_sensor(pin=PIR_PIN):
    """
    Sets up the GPIO pin for the PIR sensor.
    Args:
        pin (int): GPIO pin number to which the PIR sensor is connected.
    """
    try:
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setup(pin, GPIO.IN)
        print(f"PIR sensor setup on GPIO pin {pin}")
    except Exception as e:
        print(f"Error setting up PIR sensor: {e}")

def read_pir_sensor(pin=PIR_PIN):
    """
    Reads the state of the PIR sensor.
    Args:
        pin (int): GPIO pin number to read from.
    Returns:
        int: 1 if motion is detected, 0 otherwise.
    """
    try:
        return GPIO.input(pin)
    except Exception as e:
        print(f"Error reading PIR sensor: {e}")
        return 0

def calibrate_pir_sensor(sample_time=10):
    """
    Calibrates the PIR sensor by reading background noise levels.
    Args:
        sample_time (int): Time in seconds to sample the sensor.
    Returns:
        float: Average background noise level.
    """
    setup_pir_sensor()
    print(f"Calibrating PIR sensor for {sample_time} seconds...")
    start_time = time.time()
    readings = []
    while time.time() - start_time < sample_time:
        readings.append(read_pir_sensor())
        time.sleep(0.1)
    #GPIO.cleanup(PIR_PIN)
    average_noise = sum(readings) / len(readings)
    print(f"PIR sensor calibration complete. Average noise level: {average_noise}")
    return average_noise

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

def pir_sensor_main(pin=PIR_PIN, sample_time=10, threshold=0.5):
    """
    Main function to read and process PIR sensor data, including calibration and totband calculation.
    Args:
        pin (int): GPIO pin number to which the PIR sensor is connected.
        sample_time (int): Time in seconds to sample the sensor for calibration.
        threshold (float): Threshold value for the totband calculation.
    Returns:
        None
    """
    try:
        setup_pir_sensor(pin)
        average_noise = calibrate_pir_sensor(sample_time)
        print("Monitoring PIR sensor...")
        while True:
            motion_detected = read_pir_sensor(pin)
            totband_value = calculate_totband(motion_detected - average_noise, threshold)

            if totband_value != 0:
                print("Motion Detected!")
            else:
                print("No significant motion.")

            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting PIR sensor monitoring.")
    finally:
        GPIO.cleanup(pin)  # Reset GPIO pin on exit

if __name__ == "__main__":
    try:
        pir_sensor_main()
    except RuntimeError as e:
        print(e)
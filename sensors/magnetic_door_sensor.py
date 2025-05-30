import RPi.GPIO as GPIO
import time

# Define GPIO pin for magnetic door sensor
MAGNETIC_SENSOR_PIN = 8  # Example: GPIO 18 (BCM numbering)

def setup_magnetic_sensor(pin=MAGNETIC_SENSOR_PIN):
    """
    Sets up the GPIO pin for the magnetic door sensor.
    Args:
        pin (int): GPIO pin number to which the magnetic sensor is connected.
    """
    try:
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Use pull-up resistor
        print(f"Magnetic sensor setup on GPIO pin {pin}")
    except Exception as e:
        print(f"Error setting up magnetic sensor: {e}")

def read_magnetic_sensor(pin=MAGNETIC_SENSOR_PIN):
    """
    Reads the state of the magnetic door sensor.
    Args:
        pin (int): GPIO pin number to read from.
    Returns:
        int: 1 if the door is closed, 0 if the door is open.
    """
    try:
        return GPIO.input(pin)
    except Exception as e:
        print(f"Error reading magnetic sensor: {e}")
        return None

def calibrate_magnetic_sensor(sample_time=5):
    """
    Calibrates the magnetic sensor by reading the sensor state over a period of time.
    Args:
        sample_time (int): Time in seconds to sample the sensor.
    Returns:
        float: Average sensor reading during calibration.
    """
    setup_magnetic_sensor()
    print(f"Calibrating magnetic sensor for {sample_time} seconds...")
    start_time = time.time()
    readings = []
    while time.time() - start_time < sample_time:
        readings.append(read_magnetic_sensor())
        time.sleep(0.1)
    #GPIO.cleanup(MAGNETIC_SENSOR_PIN)
    average_reading = sum(readings) / len(readings)
    print(f"Magnetic sensor calibration complete. Average reading: {average_reading}")
    return average_reading

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

def magnetic_sensor_main(pin=MAGNETIC_SENSOR_PIN, sample_time=5, threshold=0.1):
    """
    Main function to read and process magnetic door sensor data, including calibration and totband calculation.
    Args:
        pin (int): GPIO pin number to which the magnetic sensor is connected.
        sample_time (int): Time in seconds to sample the sensor for calibration.
        threshold (float): Threshold value for the totband calculation.
    """
    try:
        setup_magnetic_sensor(pin)
        average_reading = calibrate_magnetic_sensor(sample_time)
        print("Monitoring magnetic sensor...")
        while True:
            door_state = read_magnetic_sensor(pin)

            if door_state is not None:
                totband_value = calculate_totband(door_state - average_reading, threshold)

                if totband_value == 0:
                    print("Door is closed.")
                else:
                    print("Door is open!")
            else:
                print("Could not read door state.")

            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting magnetic sensor monitoring.")
    finally:
        GPIO.cleanup(pin)  # Reset GPIO pin on exit

if __name__ == "__main__":
    try:
        magnetic_sensor_main()
    except RuntimeError as e:
        print(e)
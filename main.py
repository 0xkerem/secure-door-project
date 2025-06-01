import pigpio
from sensors.rfid_reader import RFIDReader
from actuators.servo_control import ServoController
from utils import gpio_pins
from sensors.pir_sensor import setup_pir_sensor, read_pir_sensor
from sensors.ultrasonic_sensor import UltrasonicSensor
import time

PIR_PIN = gpio_pins.PIR_SENSOR_PIN

# Create a shared pigpio.pi() instance for all hardware
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon! Is pigpiod running?")

# Instantiate ultrasonic sensor globally so it can be used in callback
ultrasonic_sensor = UltrasonicSensor(pi=pi)  # Use default pins; adjust if needed

def activate_camera(gpio, level, tick):
    print("Camera activated due to motion detection.")
    # Add actual camera activation code here

    # Get and print the distance when motion is detected
    distance = ultrasonic_sensor.get_distance()
    if distance is not None:
        print(f"Distance measured: {distance:.1f} cm")
    else:
        print("Distance measurement failed.")

def main():
    authorized_uids = ["0C00201B99", "0C00203733"]
    reader = RFIDReader(serial_port="/dev/serial0", authorized_uids=authorized_uids)
    servo = ServoController(pin=gpio_pins.SERVO_PIN)

    setup_pir_sensor(pi, PIR_PIN)

    # Set up PIR event callback using pigpio's callback
    pir_callback = pi.callback(PIR_PIN, pigpio.RISING_EDGE, activate_camera)

    print("Place your RFID card near the reader...")

    try:
        while True:
            authorized, uid = reader.read_card()
            if uid:
                print(f"Card UID: {uid} - {'AUTHORIZED' if authorized else 'UNAUTHORIZED'}")
                if authorized:
                    print("Access granted - Opening door.")
                    servo.set_angle(180)
                    time.sleep(2)
                    servo.set_angle(0)
                    print("Please remove the card...")
                    while True:
                        _, current_uid = reader.read_card()
                        if not current_uid:
                            break
                        time.sleep(0.1)
                else:
                    print("Access denied!")
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        reader.cleanup()
        servo.cleanup()
        ultrasonic_sensor.cleanup()
        pir_callback.cancel()
        pi.stop()

if __name__ == "__main__":
    main()
# main.py

import RPi.GPIO as GPIO
from sensors.rfid_reader import RFIDReader
from actuators.servo_control import ServoController
from utils import gpio_pins
from sensors.pir_sensor import setup_pir_sensor
import time

PIR_PIN = gpio_pins.PIR_SENSOR_PIN

def activate_camera(channel):
    print("Camera activated due to motion detection.")
    # Add actual camera activation code here

def main():
    GPIO.setmode(GPIO.BCM)  # Set mode ONCE here

    authorized_uids = ["0C00201B99", "0C00203733"]
    reader = RFIDReader(serial_port="/dev/serial0", authorized_uids=authorized_uids)
    servo = ServoController(pin=gpio_pins.SERVO_PIN)

    setup_pir_sensor()  # This should just do GPIO.setup(PIR_PIN, GPIO.IN)

    GPIO.add_event_detect(PIR_PIN, GPIO.RISING, callback=activate_camera)

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
        GPIO.cleanup()

if __name__ == "__main__":
    main()
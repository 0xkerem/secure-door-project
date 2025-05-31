from sensors.rfid_reader import RFIDReader
from actuators.servo_control import ServoController
from utils import gpio_pins
import time

def main():
    # List of authorized RFID UIDs
    authorized_uids = ["0C00201B99", "0C00203733"]

    # Initialize RFID reader
    reader = RFIDReader(serial_port="/dev/serial0", authorized_uids=authorized_uids)
    # Initialize ServoController on GPIO17 (orange wire)
    servo = ServoController(pin=gpio_pins.SERVO_PIN)

    print("Place your RFID card near the reader...")

    try:
        while True:
            authorized, uid = reader.read_card()
            if uid:
                print(f"Card UID: {uid} - {'AUTHORIZED' if authorized else 'UNAUTHORIZED'}")
                if authorized:
                    print("Access granted - Opening door.")
                    servo.set_angle(180)   # Open door
                    time.sleep(2)         # Keep door open for 2 seconds
                    servo.set_angle(0)    # Close door
                    # Wait until card is removed
                    print("Please remove the card...")
                    while True:
                        _, current_uid = reader.read_card()
                        if not current_uid:  # No card detected
                            break
                        time.sleep(0.1)  # Small delay to avoid busy waiting
                else:
                    print("Access denied!")
                time.sleep(1)  # Debounce delay
    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        reader.cleanup()
        servo.cleanup()

if __name__ == "__main__":
    main()
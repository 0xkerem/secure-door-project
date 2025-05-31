# main.py

from sensors.rfid_reader import RFIDReader
import time

def main():
    # Add authorized UIDs as needed
    authorized_uids = ["0C00201B99", "0C00203733"]

    # Initialize RFID reader
    reader = RFIDReader(serial_port="/dev/serial0", authorized_uids=authorized_uids)

    print("Place your RFID card near the reader...")

    try:
        while True:
            authorized, uid = reader.read_card()
            if uid:
                print(f"Card UID: {uid} - {'AUTHORIZED' if authorized else 'UNAUTHORIZED'}")
                time.sleep(1)  # Debounce delay
    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        reader.cleanup()

if __name__ == "__main__":
    main()

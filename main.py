import pigpio
import threading
import time
from queue import Queue, Empty

from sensors.rfid_reader import RFIDReader
from actuators.servo_control import ServoController
from utils import gpio_pins
from sensors.pir_sensor import setup_pir_sensor
from sensors.ultrasonic_sensor import UltrasonicSensor
from sensors.magnetic_door_sensor import MagneticSensor
from pi_sender import send_status_async
from camera.mqtt_pub import send_image  # Make sure mqtt_pub.py is accessible/importable

PIR_PIN = gpio_pins.PIR_SENSOR_PIN
MAGNETIC_PIN = gpio_pins.MAGNETIC_DOOR_SENSOR_PIN

# Create a shared pigpio.pi() instance for all hardware
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon! Is pigpiod running?")

# Instantiate ultrasonic & magnetic sensors globally so they can be used in callbacks/threads
ultrasonic_sensor = UltrasonicSensor(pi=pi)
magnetic_sensor = MagneticSensor(pi=pi, pin=MAGNETIC_PIN)

# Globals and synchronization primitives
door_open_alarm_triggered = False
authorized_door_open = False
last_motion_time = 0  # Track last motion detection time
shutdown_event = threading.Event()
rfid_queue = Queue()


def pir_motion_callback(gpio, level, tick):
    """Called by pigpio when PIR sensor goes high."""
    global last_motion_time
    # Update the timestamp for the most recent motion
    last_motion_time = time.time()


def monitor_magnetic_sensor():
    """
    Checks if the door is open without authorized access.
    If triggered, sends PHYSICAL_ALARM asynchronously.
    Should be called frequently (e.g., in a loop or separate thread).
    """
    global door_open_alarm_triggered, authorized_door_open

    door_state = magnetic_sensor.read()  # 1 = closed, 0 = open

    if door_state == 0 and not authorized_door_open:
        if not door_open_alarm_triggered:
            print("ALARM: Door opened WITHOUT authorization!")
            try:
                send_status_async("PHYSICAL_ALARM")
            except Exception as e:
                print(f"Failed to send PHYSICAL_ALARM status: {e}")
            door_open_alarm_triggered = True

    elif door_state == 1:
        # Door closed: reset both flags
        door_open_alarm_triggered = False
        authorized_door_open = False


def rfid_reader_thread(reader: RFIDReader, queue: Queue, stop_event: threading.Event):
    """
    Worker thread that continuously attempts to read an RFID card (with a short timeout).
    Whenever a UID is read (authorized or not), it pushes (authorized, uid) into the queue.
    """
    while not stop_event.is_set():
        # Attempt to read a card, waiting at most 0.5 seconds each call
        authorized, uid = reader.read_card(timeout=0.5)
        if uid:
            queue.put((authorized, uid))
        # Loop again immediately (reader.read_card will block up to timeout)


def main():
    global authorized_door_open, last_motion_time

    # 1) Instantiate hardware interfaces
    authorized_uids = ["0C00201B99", "0C00203733"]
    reader = RFIDReader(serial_port="/dev/serial0", authorized_uids=authorized_uids)
    servo = ServoController(pin=gpio_pins.SERVO_PIN)

    # 2) Set up PIR sensor callback
    setup_pir_sensor(pi, PIR_PIN)
    pir_callback = pi.callback(PIR_PIN, pigpio.RISING_EDGE, pir_motion_callback)

    # 3) Start RFID reader thread
    rfid_thread = threading.Thread(
        target=rfid_reader_thread,
        args=(reader, rfid_queue, shutdown_event),
        daemon=True
    )
    rfid_thread.start()

    print("System initialized. Waiting for activity...")

    try:
        while True:
            # 4) --------------- Process RFID events (if any) ---------------
            try:
                # Wait up to 0.1 seconds for a new card read
                authorized, uid = rfid_queue.get(timeout=0.1)
            except Empty:
                authorized = False
                uid = None

            if uid:
                print(f"Card UID: {uid} - {'AUTHORIZED' if authorized else 'UNAUTHORIZED'}")

                if authorized:
                    print("Access granted - Opening door.")
                    try:
                        send_status_async("AUTHORIZED_CARD")
                    except Exception as e:
                        print(f"Failed to send AUTHORIZED_CARD status: {e}")

                    authorized_door_open = True
                    servo.set_angle(180)
                    time.sleep(2)
                    servo.set_angle(0)
                    print("Please remove the card...")

                    # Wait until the card is removed
                    # We keep polling the reader in short intervals:
                    while True:
                        # Attempt a quick read; if no UID returned, assume card removed
                        _, current_uid = reader.read_card(timeout=0.5)
                        monitor_magnetic_sensor()  # keep checking door while waiting
                        if not current_uid:
                            break
                        time.sleep(0.1)

                else:
                    print("Access denied!")
                    try:
                        send_status_async("UNAUTHORIZED_CARD")
                    except Exception as e:
                        print(f"Failed to send UNAUTHORIZED_CARD status: {e}")

                # Small delay so we don’t immediately re‐process the same card
                time.sleep(0.5)

            # 5) --------------- Monitor magnetic sensor ---------------
            monitor_magnetic_sensor()

            # 6) --------------- PIR + Ultrasonic “Send Image” logic ---------------
            current_time = time.time()
            if (current_time - last_motion_time) <= 5:
                distance = ultrasonic_sensor.get_distance()
                if distance is not None and distance <= 50:
                    print(f"Conditions met (distance={distance} cm, motion detected in last 5 seconds). Sending image.")
                    try:
                        send_image()
                    except Exception as e:
                        print(f"Failed to send image: {e}")
                    # Reset so we don’t retrigger continuously
                    last_motion_time = 0

            # 7) Short sleep to avoid a tight busy‐loop
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutting down gracefully...")

    finally:
        # Signal threads to stop
        shutdown_event.set()
        rfid_thread.join(timeout=1.0)

        # Cleanup hardware
        reader.cleanup()
        servo.cleanup()
        ultrasonic_sensor.cleanup()
        pir_callback.cancel()
        pi.stop()


if __name__ == "__main__":
    main()
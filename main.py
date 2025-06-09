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
from camera.mqtt_pub import send_image

import paho.mqtt.client as mqtt

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

# --- NEW: Track time of last authorized door open and allowed open interval
last_authorized_open_time = 0
AUTHORIZED_DOOR_OPEN_INTERVAL = 5  # seconds, adjust as needed

# --- Face recognition MQTT result handling ---
face_result_queue = Queue()

def on_face_result(client, userdata, msg):
    result = msg.payload.decode()
    print(f"[MQTT] Received face recognition result: {result}")
    face_result_queue.put(result)

def start_face_result_listener(broker_ip="192.168.166.195", topic="camera/result"):
    client = mqtt.Client()
    client.on_message = on_face_result
    client.connect(broker_ip, 1883, 60)
    client.subscribe(topic)
    client.loop_start()  # Run MQTT loop in background thread
    return client

# --- End face recognition MQTT section ---

def pir_motion_callback(gpio, level, tick):
    """Called by pigpio when PIR sensor goes high."""
    global last_motion_time
    # Update the timestamp for the most recent motion
    last_motion_time = time.time()

def alarm_monitor_thread(stop_event):
    global door_open_alarm_triggered, authorized_door_open
    global last_authorized_open_time

    while not stop_event.is_set():
        door_state = magnetic_sensor.read()  # 1 = closed, 0 = open
        current_time = time.time()

        # If the door is open and NOT within the authorized open window, trigger the alarm
        if door_state == 0:
            # Check if the door was opened legally (recent authorized open)
            if not authorized_door_open:
                # If not currently in authorized door open interval, trigger alarm
                if (current_time - last_authorized_open_time) > AUTHORIZED_DOOR_OPEN_INTERVAL:
                    if not door_open_alarm_triggered:
                        print("ALARM: Door opened WITHOUT authorization!")
                        try:
                            send_status_async("PHYSICAL_ALARM")
                        except Exception as e:
                            print(f"Failed to send PHYSICAL_ALARM status: {e}")
                        door_open_alarm_triggered = True
            else:
                # Door is open and authorized, do not trigger alarm
                pass
        elif door_state == 1:
            # Door closed: reset the flags
            door_open_alarm_triggered = False
            authorized_door_open = False

        # Sleep to avoid busy-waiting
        time.sleep(0.1)

def rfid_reader_thread(reader: RFIDReader, queue: Queue, stop_event: threading.Event):
    """
    Worker thread that continuously attempts to read an RFID card (with a short timeout).
    Whenever a UID is read (authorized or not), it pushes (authorized, uid) into the queue.
    """
    while not stop_event.is_set():
        # Attempt to read a card, waiting at most 0.5 seconds each call
        authorized, uid = reader.read_card()
        if uid:
            queue.put((authorized, uid))
        # Loop again immediately (reader.read_card will block up to timeout)

def process_rfid_events(reader, servo):
    """
    Process RFID events from the rfid_queue.
    """
    global authorized_door_open, door_open_alarm_triggered
    global last_authorized_open_time

    try:
        authorized, uid = rfid_queue.get_nowait()
    except Empty:
        return

    if uid:
        print(f"Card UID: {uid} - {'AUTHORIZED' if authorized else 'UNAUTHORIZED'}")

        if authorized:
            print("Access granted - Opening door.")
            try:
                send_status_async("AUTHORIZED_CARD")
            except Exception as e:
                print(f"Failed to send AUTHORIZED_CARD status: {e}")

            authorized_door_open = True
            last_authorized_open_time = time.time()  # Record time for legal open
            servo.set_angle(180)
            time.sleep(2)
            servo.set_angle(0)
            print("Please remove the card...")

            # Wait until the card is removed
            while True:
                _, current_uid = reader.read_card()
                if not current_uid:
                    break
                time.sleep(0.1)

        else:
            print("Access denied!")
            try:
                send_status_async("UNAUTHORIZED_CARD")
            except Exception as e:
                print(f"Failed to send UNAUTHORIZED_CARD status: {e}")

        # Small delay to avoid re-processing the same card
        time.sleep(0.5)

def main():
    global authorized_door_open, last_motion_time, last_authorized_open_time

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

    # 4) Start alarm monitor thread
    alarm_thread = threading.Thread(
        target=alarm_monitor_thread,
        args=(shutdown_event,),
        daemon=True
    )
    alarm_thread.start()

    # 5) Start listening for face recognition results
    face_result_mqtt_client = start_face_result_listener(broker_ip="192.168.166.195")  # Change if broker is not local

    print("System initialized. Waiting for activity...")

    try:
        while True:
            # --- Process RFID events (if any) ---
            process_rfid_events(reader, servo)

            # --- PIR + Ultrasonic “Send Image” logic ---
            current_time = time.time()
            if (current_time - last_motion_time) <= 5:
                distance = ultrasonic_sensor.get_distance()
                if distance is not None and distance <= 50:
                    print(f"Conditions met (distance={distance} cm, motion detected in last 5 seconds). Sending image.")
                    try:
                        send_image()
                        print("Waiting for face recognition result...")

                        # Poll for MQTT face result for up to 5s
                        wait_start = time.time()
                        result = None
                        while (time.time() - wait_start) < 5:
                            try:
                                result = face_result_queue.get(timeout=0.1)
                                break
                            except Empty:
                                # Continue polling RFID events
                                process_rfid_events(reader, servo)

                        if result is not None and result != "unknown":
                            print("Face recognized! Activating servo.")
                            authorized_door_open = True
                            last_authorized_open_time = time.time()  # Record time for legal open
                            try:
                                send_status_async("AUTHORIZED_CARD")
                            except Exception as e:
                                print(f"Failed to send AUTHORIZED_CARD status: {e}")
                            servo.set_angle(180)
                            time.sleep(2)
                            servo.set_angle(0)
                        elif result == "unknown":
                            print("No face match detected.")
                        else:
                            print("No result received from face recognition in time.")
                    except Exception as e:
                        print(f"Failed to send image: {e}")

                    # Reset to avoid continuous retriggering
                    last_motion_time = 0

            # --- Short sleep to avoid a tight busy-loop ---
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutting down gracefully...")

    finally:
        # Signal threads to stop
        shutdown_event.set()
        rfid_thread.join(timeout=1.0)
        alarm_thread.join(timeout=1.0)

        # Cleanup hardware
        reader.cleanup()
        servo.cleanup()
        ultrasonic_sensor.cleanup()
        pir_callback.cancel()
        pi.stop()

        # Cleanup MQTT face result listener
        face_result_mqtt_client.loop_stop()
        face_result_mqtt_client.disconnect()

if __name__ == "__main__":
    main()
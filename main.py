import pigpio
from sensors.rfid_reader import RFIDReader
from actuators.servo_control import ServoController
from utils import gpio_pins
from sensors.pir_sensor import setup_pir_sensor
from sensors.ultrasonic_sensor import UltrasonicSensor
from sensors.magnetic_door_sensor import MagneticSensor
from pi_sender import send_status_async
import time

PIR_PIN = gpio_pins.PIR_SENSOR_PIN
MAGNETIC_PIN = gpio_pins.MAGNETIC_DOOR_SENSOR_PIN

# Create a shared pigpio.pi() instance for all hardware
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon! Is pigpiod running?")

# Instantiate ultrasonic sensor globally so it can be used in callback
ultrasonic_sensor = UltrasonicSensor(pi=pi)  # Use default pins; adjust if needed
magnetic_sensor = MagneticSensor(pi=pi, pin=MAGNETIC_PIN)  # Magnetic sensor instance

door_open_alarm_triggered = False
authorized_door_open = False

def activate_camera(gpio, level, tick):
    print("Camera activated due to motion detection.")
    # Add actual camera activation code here

    # Get and print the distance when motion is detected
    distance = ultrasonic_sensor.get_distance()
    if distance is not None:
        print(f"Distance measured: {distance:.1f} cm")
    else:
        print("Distance measurement failed.")

def monitor_magnetic_sensor():
    """
    Checks if the door is open without authorized access (card or face).
    If triggered, sends PHYSICAL_ALARM to ESP32 and prints alarm.
    Should be called in main loop.
    """
    global door_open_alarm_triggered, authorized_door_open
    door_state = magnetic_sensor.read()  # 1 = closed, 0 = open

    if door_state == 0 and not authorized_door_open:
        if not door_open_alarm_triggered:
            print("ALARM: Door opened WITHOUT authorization!")
            # Send PHYSICAL_ALARM alert over WiFi (now async)
            try:
                send_status_async("PHYSICAL_ALARM")
            except Exception as e:
                print(f"Failed to send PHYSICAL_ALARM status: {e}")
            door_open_alarm_triggered = True

    elif door_state == 1:
        # Door closed: reset both flags
        door_open_alarm_triggered = False
        authorized_door_open = False

def main():
    global authorized_door_open
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
            monitor_magnetic_sensor()

            if uid:
                print(f"Card UID: {uid} - {'AUTHORIZED' if authorized else 'UNAUTHORIZED'}")
                if authorized:
                    print("Access granted - Opening door.")
                    # Send AUTHORIZED_CARD to ESP32 (now async)
                    try:
                        send_status_async("AUTHORIZED_CARD")
                    except Exception as e:
                        print(f"Failed to send AUTHORIZED_CARD status: {e}")

                    authorized_door_open = True
                    servo.set_angle(180)
                    time.sleep(2)
                    servo.set_angle(0)
                    print("Please remove the card...")
                    # Wait until card is removed
                    while True:
                        _, current_uid = reader.read_card()
                        monitor_magnetic_sensor()
                        if not current_uid:
                            break
                        time.sleep(0.1)
                else:
                    print("Access denied!")
                    # Send UNAUTHORIZED_CARD to ESP32 (now async)
                    try:
                        send_status_async("UNAUTHORIZED_CARD")
                    except Exception as e:
                        print(f"Failed to send UNAUTHORIZED_CARD status: {e}")
                time.sleep(1)
            else:
                # No card, just monitor door
                monitor_magnetic_sensor()
                time.sleep(0.1)
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
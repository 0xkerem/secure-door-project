# Secure Door System

A comprehensive and modular Raspberry Pi-based security system that automates access control using RFID, face recognition, motion detection, magnetic and ultrasonic sensors, and real-time alerting via ESP32. Camera images are transmitted via MQTT to an external face detection system.

## Overview

The Secure Door System is designed to monitor and control physical access to a restricted area. It integrates multiple sensors and actuators to detect, identify, and react to authorized and unauthorized access attempts. Alerts and status notifications are sent in real time to an ESP32 module, which handles physical alarms (LCD, LED, buzzer, etc.). Camera images are published over MQTT to a separate system that performs face recognition using `deepface`.

### Key Features

* RFID Authentication using RC522
* Face Recognition powered externally via DeepFace
* Motion Detection via PIR sensors
* Door Status Monitoring via magnetic sensors
* Proximity Sensing via ultrasonic sensor
* Servo-Controlled Lock Mechanism
* MQTT for image transmission
* ESP32 Alert System via TCP sockets

## Project Structure

```
secure-door-system/
├── actuators/                 # Servo control logic
├── camera/                    # Image capture and calibration
├── secure_door_warnings/     # ESP32 project (PlatformIO)
├── sensors/                   # RFID, PIR, Magnetic, Ultrasonic
├── utils/                     # Logging, GPIO pin mapping, filters
├── main.py                    # Main control logic (entry point)
├── pi_sender.py               # TCP client to communicate with ESP32
├── requirements.txt           # Python dependencies
└── README.md
```

## Hardware Used

* Raspberry Pi 3B+
* ESP32 development board
* RFID Reader (RC522)
* PIR Motion Sensor (HC-SR501)
* Magnetic Reed Switch
* Ultrasonic Sensor (HC-SR04)
* Servo Motor (SG90)
* USB or Pi Camera Module
* Power Supply, Breadboard, Jumper Wires
* OLED screen, LED, Buzzer

## Software Stack

### Raspberry Pi (Python)

* Python 3
* `deepface`, `opencv-python`
* `paho-mqtt`, `pigpio`, `numpy`, `scipy`

### ESP32 (C++ via PlatformIO)

* TCP server implementation for alert reception
* LED/Buzzer control based on received commands

## Communication

* TCP/IP: Between Raspberry Pi (`pi_sender.py`) and ESP32 (C++) for alerts
* MQTT: For publishing camera images to the external face recognition system

### Message Examples (TCP to ESP32)

| Event             | Message Sent       |
| ----------------- | ------------------ |
| Authorized RFID   | AUTHORIZED\_CARD   |
| Unauthorized RFID | UNAUTHORIZED\_CARD |
| Door forced open  | PHYSICAL\_ALARM    |

## Getting Started

1. Install dependencies on Raspberry Pi:

```bash
pip install -r requirements.txt
```

2. Run the system:

```bash
python main.py
```

3. Flash ESP32 using [PlatformIO](https://platformio.org/):

```bash
cd secure_door_warnings
platformio run --target upload
```

4. Connect and test your sensors.

## Face Recognition Setup

1. Configure the external face detection server to subscribe to the MQTT topic.
2. Ensure it uses `deepface` to process incoming images.
3. Store known faces appropriately for matching.

## License

This project is licensed under the Apache License 2.0. See `LICENSE` for details.

## Contributions

Pull requests are welcome. Feel free to fork and suggest improvements.

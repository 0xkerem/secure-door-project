# mqtt_pub.py
import cv2
import base64
import paho.mqtt.client as mqtt
import numpy as np

def send_image(broker_ip="192.168.166.195", topic="camera/image", calib_path="camera/calibration_result.npz"):
    # Load calibration data
    try:
        data = np.load(calib_path)
        cameraMatrix = data['cameraMatrix']
        distCoeffs = data['distCoeffs']
    except Exception as e:
        print(f"Failed to load calibration data: {e}")
        return

    # Open the camera
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture image from camera.")
        return

    # Undistort the frame using calibration data
    undistorted = cv2.undistort(frame, cameraMatrix, distCoeffs)

    # Encode the undistorted image data
    _, buffer = cv2.imencode('.jpg', undistorted)
    jpg_as_text = base64.b64encode(buffer).decode()

    # Establish MQTT connection
    client = mqtt.Client()
    client.connect(broker_ip, 1883, 60)

    # Publish message
    client.publish(topic, jpg_as_text)
    client.disconnect()

    print("Image sent successfully (with calibration applied).")
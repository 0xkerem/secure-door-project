import cv2
import base64
import paho.mqtt.client as mqtt
import numpy as np
import os
import glob
import random

def send_image(broker_ip="192.168.166.195", topic="camera/image", calib_path="camera/calibration_result.npz",
               test_img_dir="camera/img", use_test_image=True):
    # Load calibration data
    try:
        data = np.load(calib_path)
        cameraMatrix = data['cameraMatrix']
        distCoeffs = data['distCoeffs']
    except Exception as e:
        print(f"Failed to load calibration data: {e}")
        return

    # For test purposes, use a random static image instead of camera frame
    if use_test_image:
        img_files = glob.glob(os.path.join(test_img_dir, "test*.jpg"))
        if not img_files:
            print(f"No test images found in {test_img_dir}")
            return
        test_img_path = random.choice(img_files)
        frame = cv2.imread(test_img_path)
        if frame is None:
            print(f"Failed to load test image from {test_img_path}")
            return
        ret = True
    else:
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        if not ret:
            print("Failed to capture image from camera.")
            return

    # Undistort the frame using calibration data
    undistorted = cv2.undistort(frame, cameraMatrix, distCoeffs)
    
    # Keep aspect ratio (original is 1920x1080)
    original_height, original_width = undistorted.shape[:2]
    new_width = 160
    new_height = int(original_height * new_width / original_width)
    frame = cv2.resize(undistorted, (new_width, new_height))

    # Encode the undistorted image data
    _, buffer = cv2.imencode('.jpg', frame)
    jpg_as_text = base64.b64encode(buffer).decode()

    # Establish MQTT connection
    client = mqtt.Client()
    client.connect(broker_ip, 1883, 60)
    # Publish message
    client.publish(topic, jpg_as_text)
    client.disconnect()

    print(f"Image {test_img_path} sent successfully (with calibration applied). Now deleting it.")

    # Delete the image so it is not sent again
    try:
        os.remove(test_img_path)
        print(f"Deleted {test_img_path}")
    except Exception as e:
        print(f"Failed to delete {test_img_path}: {e}")
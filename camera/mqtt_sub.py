# mqtt_sub.py
import base64
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import time

def _on_message(client, userdata, msg):
    print("Image received, processing...")
    try:
        img_data = base64.b64decode(msg.payload)
        np_arr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            print("Error: Decoded image is None.")
            return
        filename = f"received_{int(time.time())}.jpg"
        cv2.imwrite(filename, img)
        print(f"Image saved: {filename}")
    except Exception as e:
        print("Failed to process image:", e)

def start_listening(broker_ip="localhost", topic="camera/image"):
    client = mqtt.Client()
    client.on_message = _on_message

    client.connect(broker_ip, 1883, 60)
    client.subscribe(topic)

    print("Started listening...")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("Stopping listener...")
        client.loop_stop()
        client.disconnect()

def stop_listening(client):
    client.loop_stop()   
    client.disconnect()
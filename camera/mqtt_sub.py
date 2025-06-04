# mqtt_sub.py
import base64
import cv2
import numpy as np
import paho.mqtt.client as mqtt

def _on_message(client, userdata, msg):
    print("Fotoğraf alındı, işleniyor...")
    img_data = base64.b64decode(msg.payload)
    np_arr = np.frombuffer(img_data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Saving the image
    cv2.imwrite("received.jpg", img)
    print("Fotoğraf kaydedildi: received.jpg")

def start_listening(broker_ip="localhost", topic="camera/image"):
    client = mqtt.Client()
    client.on_message = _on_message

    client.connect(broker_ip, 1883, 60)
    client.subscribe(topic)

    print("Dinlemeye başlandı...")
    client.loop_forever()

def stop_listening():
    client.loop_stop()   
    client.disconnect()

#call start_listening function from main


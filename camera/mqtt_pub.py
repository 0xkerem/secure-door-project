# mqtt_pub.py
import cv2
import base64
import paho.mqtt.client as mqtt

def send_image(broker_ip="localhost", topic="camera/image"):
    # Kamera aç
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Kamera görüntüsü alınamadı.")
        return

    # Encoding the image data
    _, buffer = cv2.imencode('.jpg', frame)
    jpg_as_text = base64.b64encode(buffer).decode()

    # Establishing MQTT connection
    client = mqtt.Client()
    client.connect(broker_ip, 1883, 60)

    # Sending message
    client.publish(topic, jpg_as_text)
    client.disconnect()

    print("📤 Görüntü başarıyla gönderildi.")


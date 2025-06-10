import cv2
import base64
import paho.mqtt.client as mqtt
import numpy as np
import os
import glob
import random

def send_image(broker_ip="192.168.166.195", topic="camera/image", calib_path="camera/calibration_result.npz",
               test_img_dir="camera/img", use_test_image=False):
    """
    Send an image via MQTT, either from camera or test image.
    
    Args:
        broker_ip: MQTT broker IP address
        topic: MQTT topic to publish to
        calib_path: Path to camera calibration data
        test_img_dir: Directory containing test images
        use_test_image: If True, use random test image; if False, use camera
    """
    # Load calibration data
    try:
        data = np.load(calib_path)
        cameraMatrix = data['cameraMatrix']
        distCoeffs = data['distCoeffs']
        print("Camera calibration data loaded successfully.")
    except Exception as e:
        print(f"Failed to load calibration data: {e}")
        return False

    # Initialize variables
    frame = None
    test_img_path = None
    
    # Get image source
    if use_test_image:
        # Use test image
        try:
            img_files = glob.glob(os.path.join(test_img_dir, "test*.jpg"))
            if not img_files:
                print(f"No test images found in {test_img_dir}")
                return False
            
            test_img_path = random.choice(img_files)
            frame = cv2.imread(test_img_path)
            
            if frame is None:
                print(f"Failed to load test image from {test_img_path}")
                return False
                
            print(f"Using test image: {test_img_path}")
            
        except Exception as e:
            print(f"Error loading test image: {e}")
            return False
    else:
        # Use camera
        cap = None
        try:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Failed to open camera.")
                return False
                
            # Set camera properties for better capture
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Capture frame
            ret, frame = cap.read()
            
            if not ret or frame is None:
                print("Failed to capture image from camera.")
                return False
                
            print("Image captured from camera successfully.")
            
        except Exception as e:
            print(f"Error capturing from camera: {e}")
            return False
        finally:
            if cap is not None:
                cap.release()

    # Process the image
    try:
        # Undistort the frame using calibration data
        undistorted = cv2.undistort(frame, cameraMatrix, distCoeffs)
        
        # Keep aspect ratio (original is 1920x1080)
        original_height, original_width = undistorted.shape[:2]
        new_width = 160
        new_height = int(original_height * new_width / original_width)
        
        # Resize image
        resized_frame = cv2.resize(undistorted, (new_width, new_height))
        
        print(f"Image processed: {original_width}x{original_height} -> {new_width}x{new_height}")
        
    except Exception as e:
        print(f"Error processing image: {e}")
        return False

    # Encode and send image
    try:
        # Encode the processed image
        _, buffer = cv2.imencode('.jpg', resized_frame)
        jpg_as_text = base64.b64encode(buffer).decode()
        
        # Establish MQTT connection
        client = mqtt.Client()
        client.connect(broker_ip, 1883, 60)
        
        # Publish message
        result = client.publish(topic, jpg_as_text)
        client.disconnect()
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            if use_test_image and test_img_path:
                print(f"Test image {test_img_path} sent successfully (with calibration applied).")
                
                # Delete the test image so it is not sent again
                try:
                    os.remove(test_img_path)
                    print(f"Deleted {test_img_path}")
                except Exception as e:
                    print(f"Failed to delete {test_img_path}: {e}")
            else:
                print("Camera image sent successfully (with calibration applied).")
            return True
        else:
            print(f"Failed to publish MQTT message. Error code: {result.rc}")
            return False
            
    except Exception as e:
        print(f"Error sending image via MQTT: {e}")
        return False

# Additional utility function for debugging
def test_camera_capture():
    """Test camera capture without MQTT sending."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return False
    
    ret, frame = cap.read()
    cap.release()
    
    if ret and frame is not None:
        print(f"Camera test successful. Frame shape: {frame.shape}")
        return True
    else:
        print("Camera test failed")
        return False

# Additional utility function for testing
def list_test_images(test_img_dir="camera/img"):
    """List available test images."""
    img_files = glob.glob(os.path.join(test_img_dir, "test*.jpg"))
    print(f"Found {len(img_files)} test images:")
    for img in img_files:
        print(f"  - {img}")
    return img_files

if __name__ == "__main__":
    # Test the function
    print("Testing camera capture...")
    test_camera_capture()
    
    print("\nListing test images...")
    list_test_images()
    
    print("\nTesting image send (camera)...")
    send_image(use_test_image=False)
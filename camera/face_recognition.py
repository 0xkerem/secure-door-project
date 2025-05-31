# face_recognition.py
from deepface import DeepFace
import os

# Path to the face database (images of known individuals)
DB_PATH = r"C:\Users\meren\Desktop\Photos" # path will be updated

# Face recognition model and settings
MODEL_NAME = "VGG-Face"              
DETECTOR_BACKEND = "opencv"          
DISTANCE_METRIC = "cosine"           


def face_recognition(image_path):
    """
    Perform face recognition for a single image.
    
    Parameters:
        image_path (str): The full path of the image to test.
    
    Returns:
        str or None: The identified person's label if a match is found, else None.
    """
    img_name = os.path.basename(image_path)
    print(f"\n🔍 Testing image: {img_name}")

    try:
        result = DeepFace.find(
            img_path=image_path,
            db_path=DB_PATH,
            model_name=MODEL_NAME,
            enforce_detection=True,
            detector_backend=DETECTOR_BACKEND,
            distance_metric=DISTANCE_METRIC
        )

        df = result[0]
        if df.empty:
            message = "❌ No match found."
            print(message)
            return None

        # Get the best match (first row)
        match = df.iloc[0]
        identity_path = match.get("identity", "Unknown")
        
        # Extract person name from the folder name
        person_label = os.path.basename(os.path.dirname(identity_path))

        message = f"✅ Matched person: {person_label}"
        print(message)
        return person_label

    except Exception as e:
        error_message = f"⚠️ Error occurred: {e}"
        print(error_message)
        return None


# Example usage: recognizing multiple test images
if __name__ == "__main__":
    face_recognition(r"C:\Users\meren\Desktop\Test\WIN_20250530_23_49_36_Pro.jpg")
    face_recognition(r"C:\Users\meren\Desktop\Test\A8.jpg")
    face_recognition(r"C:\Users\meren\Desktop\Test\WIN_20250527_16_07_08_Pro.jpg")
    face_recognition(r"C:\Users\meren\Desktop\Test\WIN_20250527_16_09_25_Pro (2).jpg")



import os
import pickle
import numpy as np
from deepface import DeepFace
from math_helpers import cosine_similarity, normalize

# Load previously created and saved embeddings
with open(r"camera/faces.pkl", "rb") as f:
    embeddings = pickle.load(f)

def get_embedding(image_path):
    """
    Extract and normalize the face embedding vector from an image using DeepFace.
    """
    emb = DeepFace.represent(img_path=image_path, model_name="VGG-Face")[0]["embedding"]
    emb = normalize(np.array(emb).reshape(1, -1))[0]
    return emb.reshape(1, -1)

def find_best_match(new_embedding, threshold=0.4):
    """
    Compare a new face embedding with stored embeddings to find the best match based on cosine similarity.
    """
    max_sim = -1
    best_person = None
    for person_name, emb in embeddings.items():
        sim = cosine_similarity(new_embedding, emb.reshape(1, -1))[0][0]
        if sim > max_sim:
            max_sim = sim
            best_person = person_name
    if max_sim >= threshold:
        return best_person
    else:
        return None

def face_recognition(image_path):
    """
    Perform face recognition on a given image by extracting its embedding and finding the best match.
    """
    print(f"\nChecking: {os.path.basename(image_path)}")
    try:
        new_emb = get_embedding(image_path)
        matched_person = find_best_match(new_emb)
        if matched_person:
            print(f"Matched person: {matched_person}")
            return matched_person
        else:
            print("No match found.")
            return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

if __name__ == "__main__":
    test_images_dir = "camera/test_images"
    if not os.path.exists(test_images_dir):
        print(f"Test images directory '{test_images_dir}' does not exist.")
    else:
        # List all files in test_images_dir
        valid_exts = {".jpg", ".jpeg", ".png", ".bmp"}
        images = [f for f in os.listdir(test_images_dir)
                  if os.path.splitext(f)[1].lower() in valid_exts]
        if not images:
            print("No image files found in test_images directory.")
        for img_name in images:
            img_path = os.path.join(test_images_dir, img_name)
            face_recognition(img_path)
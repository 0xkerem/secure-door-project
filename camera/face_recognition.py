import os
import pickle
import numpy as np
from deepface import DeepFace
from sklearn.metrics.pairwise import cosine_similarity
from sklearn.preprocessing import normalize

# Load previously created and saved embeddings
with open(r"C:\Users\meren\Desktop\secure-door-project\camera\faces.pkl", "rb") as f:
    embeddings = pickle.load(f)

def get_embedding(image_path):
    """
    Extract and normalize the face embedding vector from an image using DeepFace.
    
    Parameters:
        image_path (str): Path to the image file.
    
    Returns:
        np.ndarray: Normalized embedding vector reshaped for similarity calculation.
    """

    emb = DeepFace.represent(img_path=image_path, model_name="VGG-Face")[0]["embedding"]
    emb = normalize(np.array(emb).reshape(1, -1))[0]
    return emb.reshape(1, -1)


def find_best_match(new_embedding, threshold=0.4):
    """
    Compare a new face embedding with stored embeddings to find the best match based on cosine similarity.
    
    Parameters:
        new_embedding (np.ndarray): The embedding vector of the new face (1 x N).
        threshold (float): Similarity threshold to accept a match.
    
    Returns:
        str or None: The name of the matched person if similarity >= threshold, otherwise None.
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
    
    Parameters:
        image_path (str): Path to the test image.
    
    Returns:
        str or None: The name of the matched person or None if no match is found.
    """

    print(f"\n🔍 Checking: {os.path.basename(image_path)}")
    try:
        new_emb = get_embedding(image_path)
        matched_person = find_best_match(new_emb)
        if matched_person:
            print(f"✅ Matched person: {matched_person}")
            return matched_person
        else:
            print("❌ No match found.")
            return None
    except Exception as e:
        print(f"⚠️ An error occurred: {e}")
        return None



if __name__ == "__main__":
    # List of new images to test face recognition on
    new_images = [
        r"C:\Users\meren\Desktop\Test\WIN_20250530_23_49_36_Pro.jpg",
        r"C:\Users\meren\Desktop\Test\A8.jpg",
        r"C:\Users\meren\Desktop\Test\WIN_20250527_16_07_08_Pro.jpg",
        r"C:\Users\meren\Desktop\Test\WIN_20250527_16_09_25_Pro (2).jpg",
        r"C:\Users\meren\Desktop\Test\WIN_20250527_16_03_56_Pro.jpg",
        r"C:\Users\meren\Desktop\Test\WIN_20250527_16_04_14_Pro.jpg",
        r"C:\Users\meren\Desktop\Test\WIN_20250527_16_09_11_Pro.jpg"
    ]

    for img_path in new_images:
        face_recognition(img_path)

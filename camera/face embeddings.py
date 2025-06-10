# face embeddings
import os
import cv2
import numpy as np
from deepface import DeepFace
import pickle

print("Working directory:", os.getcwd())

embeddings = {}
photos_dir = # relative path

for person_name in os.listdir(photos_dir):
    person_folder = os.path.join(photos_dir, person_name)
    if not os.path.isdir(person_folder):
        continue

    person_embeddings = []
    for image_name in os.listdir(person_folder):
        image_path = os.path.join(person_folder, image_name)
        img = cv2.imread(image_path)

        try:
            embedding = DeepFace.represent(img_path=img, model_name="VGG-Face")[0]["embedding"]
            person_embeddings.append(embedding)
        except Exception as e:
            print(f"Could not extract embedding for {image_path}:", e)

    if person_embeddings:
        mean_embedding = np.mean(person_embeddings, axis=0)
        embeddings[person_name] = mean_embedding

# After calculating all embeddings, save them
with open("faces.pkl", "wb") as f:
    pickle.dump(embeddings, f)

print("All embeddings have been saved to faces.pkl file.")

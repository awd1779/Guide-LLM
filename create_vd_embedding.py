import os
import torch
import open_clip
from PIL import Image
import chromadb
from tqdm import tqdm

def create_or_reset_collection(collection_name="area4_collection"):
    client = chromadb.PersistentClient(path="area4_db")
    try:
        client.delete_collection(collection_name)
        print(f"Existing collection '{collection_name}' deleted.")
    except ValueError:
        print(f"No existing collection named '{collection_name}' found.")
    
    collection = client.create_collection(collection_name)
    print(f"Collection '{collection_name}' created successfully.")
    return collection

def embed_images(folder_path, collection):
    device = "cuda" if torch.cuda.is_available() else "cpu"
    
    # Load the model and preprocessing transforms from Hugging Face via OpenCLIP
    model_name = 'ViT-B-32'
    pretrained_weights = 'laion2b_s34b_b79k'
    model, preprocess_train, preprocess_val = open_clip.create_model_and_transforms(
        model_name,
        pretrained=pretrained_weights
    )
    model = model.to(device)
    preprocess = preprocess_val
    
    image_files = []
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.lower().endswith(("png", "jpg", "jpeg")):
                image_files.append(os.path.join(root, file))
    
    for image_path in tqdm(image_files, desc="Embedding images"):
        image = preprocess(Image.open(image_path)).unsqueeze(0).to(device)
        with torch.no_grad():
            embedding = model.encode_image(image).cpu().numpy().flatten().tolist()
        
        # Build a compound class string:
        # Use the folder name (floor) and the image file name (without extension) as the second part.
        floor_name = os.path.basename(os.path.dirname(image_path))
        image_name = os.path.splitext(os.path.basename(image_path))[0]
        class_name = floor_name + "|" + image_name  # e.g. "First_floor|HallwayA_north"
        
        image_id = image_name  # Using the image file name as the unique id
        
        # Add to ChromaDB with the compound "class"
        collection.add(
            embeddings=[embedding],
            metadatas=[{"class": class_name, "path": image_path, "name": image_id}],
            ids=[image_id]
        )

if __name__ == "__main__":
    collection_name = "area4_collection"
    image_folder_path = "/home/ubuntu/catkin_ws/src/scripts/vdb/area4_images"  # Adjust this path as needed
    # Create or reset the master collection
    collection = create_or_reset_collection(collection_name)
    # Embed images and store them in the master DB
    embed_images(image_folder_path, collection)

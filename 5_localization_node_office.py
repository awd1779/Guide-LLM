import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
import torch
import open_clip
import numpy as np
from PIL import Image as PILImage
import chromadb
import threading

# Global flag to indicate if the script should process images
process_image_flag = False
flag_lock = threading.Lock()

def load_retrieved_images_collection(collection_name="area4_collection"):
    persist_directory = "/home/ubuntu/catkin_ws/src/scripts/area4_db"
    client = chromadb.PersistentClient(path=persist_directory)

    try:
        collection = client.get_collection(collection_name)
    except ValueError:
        rospy.logwarn(f"Collection '{collection_name}' does not exist. Creating a new collection.")
        collection = client.create_collection(collection_name)

    return collection

def embed_current_observation(pil_image, model, preprocess, device):
    img_tensor = preprocess(pil_image).unsqueeze(0).to(device)  # Apply OpenCLIP preprocessing
    with torch.no_grad():
        embedding = model.encode_image(img_tensor).cpu().numpy().flatten().tolist()
    return embedding

def calculate_similarity(embedding1, embedding2):
    return np.dot(embedding1, embedding2) / (np.linalg.norm(embedding1) * np.linalg.norm(embedding2))

def retrieve_top_similar_image(pil_image, model, preprocess, device):
    rospy.loginfo("Embedding current observation image...")
    current_observation_embedding = embed_current_observation(pil_image, model, preprocess, device)

    rospy.loginfo("Loading image collection from ChromaDB...")
    collection = load_retrieved_images_collection("area4_collection")

    try:
        retrieved_images_with_embeddings = collection.get(include=["metadatas", "embeddings"])
    except Exception as e:
        rospy.logerr(f"Error retrieving embeddings from the collection: {e}")
        return

    #if not retrieved_images_with_embeddings['embeddings']:
    if len(retrieved_images_with_embeddings['embeddings']) == 0:

        rospy.logwarn("No embeddings found in the collection.")
        return

    # Convert embeddings to a NumPy array and calculate similarities
    embeddings_matrix = np.array(retrieved_images_with_embeddings['embeddings'])
    similarities = np.dot(embeddings_matrix, current_observation_embedding) / (
        np.linalg.norm(embeddings_matrix, axis=1) * np.linalg.norm(current_observation_embedding))

    # Get the index of the top similar image
    top_index = np.argmax(similarities)
    top_similarity = similarities[top_index]
    top_image_metadata = retrieved_images_with_embeddings['metadatas'][top_index]

    if top_similarity is not None:
        image_id = top_image_metadata.get('name', 'Unknown')
        image_class = top_image_metadata.get('class', 'Unknown')
        rospy.loginfo(f"Top similarity: {top_similarity:.4f}")
        rospy.loginfo(f"Top image class: {image_class}, id: {image_id}")
        publish_similarity(top_similarity, image_class, image_id)
    else:
        rospy.loginfo("No matching images found.")

def publish_similarity(similarity_score, image_class, image_id):
    """Publish the similarity score, image class, and image id to the 'assistants_input' topic."""
    similarity_publisher = rospy.Publisher('localization_node', String, queue_size=50)
    rospy.loginfo(f"Publishing similarity score: {similarity_score}, class: {image_class}, id: {image_id}")
    rospy.sleep(2)
    similarity_message = f"Similarity score: {similarity_score:.4f}, You are at floor: {image_class}, location: {image_id}"
    similarity_publisher.publish(String(similarity_message))
    rospy.sleep(2)

def localization_callback(msg):
    global process_image_flag
    if "current_location" in msg.data:
        rospy.loginfo("Localization signal received. Starting image processing...")
        with flag_lock:
            process_image_flag = True

def image_callback(msg, model, preprocess, device):
    global process_image_flag

    if rospy.is_shutdown():
        return

    with flag_lock:
        if not process_image_flag:
            return
        process_image_flag = False

    try:
        # Convert ROS Image message to NumPy array
        np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        # Convert NumPy array to PIL image
        pil_image = PILImage.fromarray(np_image, 'RGB')

        # Perform similarity check using PIL image
        retrieve_top_similar_image(pil_image, model, preprocess, device)

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    rospy.init_node('image_processing_on_localization_signal_node')

    # Load CLIP model from Hugging Face
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model_name = 'ViT-B-32'
    pretrained_weights = 'laion2b_s34b_b79k'
    model, _, preprocess_val = open_clip.create_model_and_transforms(model_name, pretrained=pretrained_weights)
    model = model.to(device)
    preprocess = preprocess_val  # Use OpenCLIP preprocessing

    rospy.loginfo("Model loaded and ready")

    # Subscribe to the camera topic (sensor_msgs/Image)
    rospy.Subscriber('/gibson_ros/camera/rgb/image', Image, lambda msg: image_callback(msg, model, preprocess, device))

    # Subscribe to the localization query signal (std_msgs/String)
    rospy.Subscriber('chatgpt_output_localization_query', String, localization_callback)

    rospy.spin()

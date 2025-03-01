import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
import open_clip
import numpy as np
from PIL import Image as PILImage
import chromadb
import torchvision.transforms as transforms
import threading

# Global flag to indicate if the script should process images
process_image_flag = False
flag_lock = threading.Lock()

def load_retrieved_images_collection(collection_name="retrieved_images_collection"):
    persist_directory = "/home/ubuntu/catkin_ws/src/scripts/retrieved_images_db"
    client = chromadb.PersistentClient(path=persist_directory)
    try:
        collection = client.get_collection(collection_name)
        rospy.loginfo(f"Loaded retrieved images collection: {collection_name}")
    except Exception as e:
        rospy.logwarn(f"Collection '{collection_name}' does not exist. Creating a new collection. Error: {e}")
        collection = client.create_collection(collection_name)
    return collection

def preprocess_image(pil_image):
    preprocess = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    return preprocess(pil_image).unsqueeze(0)  # Add batch dimension

def embed_current_observation(pil_image, model, device):
    img_tensor = preprocess_image(pil_image).to(device)
    with torch.no_grad():
        embedding = model.encode_image(img_tensor).cpu().numpy().flatten().tolist()
    return embedding

def calculate_similarity(embedding1, embedding2):
    return np.dot(embedding1, embedding2) / (np.linalg.norm(embedding1) * np.linalg.norm(embedding2))

def retrieve_top_similar_image(pil_image, model, device):
    rospy.loginfo("Embedding current observation image...")
    current_observation_embedding = embed_current_observation(pil_image, model, device)

    rospy.loginfo("Loading retrieved images collection from ChromaDB...")
    collection = load_retrieved_images_collection("retrieved_images_collection")

    try:
        retrieved_data = collection.get(include=["metadatas", "embeddings"])
    except Exception as e:
        rospy.logerr(f"Error retrieving embeddings from the collection: {e}")
        return

    # Instead of "if not retrieved_data['embeddings']:", check the length
    if len(retrieved_data['embeddings']) == 0:
        rospy.logwarn("No embeddings found in the retrieved images collection.")
        return

    # Convert embeddings to a NumPy array and calculate similarities
    embeddings_matrix = np.array(retrieved_data['embeddings'])
    current_embedding = np.array(current_observation_embedding)
    if np.linalg.norm(current_embedding) == 0:
        rospy.logwarn("Current observation embedding norm is zero.")
        return

    similarities = np.dot(embeddings_matrix, current_embedding) / (
        np.linalg.norm(embeddings_matrix, axis=1) * np.linalg.norm(current_embedding))

    top_index = np.argmax(similarities)
    top_similarity = similarities[top_index]
    top_image_metadata = retrieved_data['metadatas'][top_index]

    if top_similarity is not None:
        image_id = top_image_metadata.get('name', 'Unknown')
        image_class = top_image_metadata.get('class', 'Unknown')
        rospy.loginfo(f"Top similarity: {top_similarity:.4f}")
        rospy.loginfo(f"Top image class: {image_class}, id: {image_id}")
        publish_similarity(top_similarity, image_class, image_id)
    else:
        rospy.loginfo("No matching images found.")

def publish_similarity(similarity_score, image_class, image_id):
    """Publish the similarity score, image class, and image id to the '/similarity_node' topic."""
    similarity_publisher = rospy.Publisher('/similarity_node', String, queue_size=50)
    rospy.loginfo(f"Publishing similarity score: {similarity_score:.4f}, class: {image_class}, id: {image_id}")
    rospy.sleep(2)
    similarity_message = f"Similarity score: {similarity_score:.4f}, floor: {image_class}, location: {image_id}"
    similarity_publisher.publish(String(similarity_message))
    rospy.sleep(2)

def navigation_query_callback(msg):
    global process_image_flag
    with flag_lock:
        if msg.data == "query_navigation_image":
            rospy.loginfo("Received 'query_navigation_image' command. Starting image processing...")
            process_image_flag = True

def image_callback(msg, model, device):
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
        # Convert the NumPy array to a PIL image
        pil_image = PILImage.fromarray(np_image, 'RGB')
        # Perform similarity check using the PIL image
        retrieve_top_similar_image(pil_image, model, device)
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    rospy.init_node('image_processing_on_navigation_query_node')

    # Load the model using the same pretrained weights as your master DB
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, _, _ = open_clip.create_model_and_transforms('ViT-B-32', pretrained='laion2b_s34b_b79k')
    model = model.to(device)
    rospy.loginfo("Model loaded and ready")

    # Subscribe to the camera topic (sensor_msgs/Image)
    rospy.Subscriber('/gibson_ros/camera/rgb/image', Image, lambda msg: image_callback(msg, model, device))
    # Subscribe to the navigation image query signal (std_msgs/String)
    rospy.Subscriber('/chatgpt_output_navigation_image_query', String, navigation_query_callback)

    rospy.spin()

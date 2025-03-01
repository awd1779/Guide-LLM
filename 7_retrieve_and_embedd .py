import rospy
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
import torch
import open_clip
from PIL import Image as PILImage
import chromadb
import json
import threading
import numpy as np
import time
import torchvision.transforms as transforms
import os
import shutil

class CombinedNode:
    def __init__(self):
        rospy.init_node('combined_node')
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Load the CLIP model and its transforms for queries
        model_name = 'ViT-B-32'
        pretrained_weights = 'laion2b_s34b_b79k'
        self.model, _, preprocess_val = open_clip.create_model_and_transforms(model_name, pretrained=pretrained_weights)
        self.model = self.model.to(self.device)
        self.preprocess_clip = preprocess_val

        # For navigation images, we use torchvision transforms
        self.preprocess_nav = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # Load master DB (original images)
        self.all_images_collection = self.load_existing_collection(
            collection_name="area4_collection",
            persist_directory="/home/ubuntu/catkin_ws/src/scripts/area4_db"
        )

        # Settings for retrieved DB (for query results)
        self.collection_base_name = "retrieved_images_collection"
        self.retrieved_db_path = "/home/ubuntu/catkin_ws/src/scripts/retrieved_images_db"
        self.active_retrieved_collection = None  # Will hold the most recent retrieved collection

        # Flags and thread lock
        self.process_image_flag = False
        self.flag_lock = threading.Lock()

        # Publishers
        self.retrieval_complete_pub = rospy.Publisher('/image_retrieval_success', String, queue_size=10)
        self.similarity_pub = rospy.Publisher('/similarity_node', String, queue_size=50)

        # Subscribers
        rospy.Subscriber('chatgpt_output_image_query', String, self.query_callback)
        rospy.Subscriber('/chatgpt_output_navigation_image_query', String, self.navigation_query_callback)
        rospy.Subscriber('/gibson_ros/camera/rgb/image', ROSImage, self.image_callback)

    def load_existing_collection(self, collection_name, persist_directory):
        """Loads an existing collection or creates a new one if not found."""
        client = chromadb.PersistentClient(path=persist_directory)
        try:
            collection = client.get_collection(name=collection_name)
            rospy.loginfo(f"Loaded existing collection '{collection_name}'.")
        except Exception as e:
            rospy.loginfo(f"Collection '{collection_name}' not found, creating a new one. Error: {e}")
            collection = client.create_collection(collection_name)
        return collection

    def create_new_retrieved_images_collection(self):
        """
        Creates a new retrieved images collection with a unique name based on the current timestamp.
        """
        unique_collection_name = f"{self.collection_base_name}_{int(time.time())}"
        client = chromadb.PersistentClient(path=self.retrieved_db_path)
        try:
            collection = client.create_collection(unique_collection_name)
            rospy.loginfo(f"Created new retrieved images collection: '{unique_collection_name}'.")
        except chromadb.errors.UniqueConstraintError as e:
            rospy.logerr(f"UniqueConstraintError during creation: {e}. Retrieving existing collection.")
            collection = client.get_collection(unique_collection_name)
        # Save this collection as the active one
        self.active_retrieved_collection = collection
        return collection

    def query_callback(self, msg):
        """
        Triggered by a query message.
        Creates a new retrieved images collection, queries the master DB,
        and stores matching images in the new collection.
        """
        query_string = msg.data
        rospy.loginfo(f"Received query: {query_string}")

        # Create a new retrieved images collection for this query
        collection = self.create_new_retrieved_images_collection()

        # Parse the query string into items
        query_items = [item.strip() for item in query_string.split(",") if item.strip()]
        rospy.loginfo(f"Parsed query items: {query_items}")

        try:
            results = self.all_images_collection.get(
                where={"class": {"$in": query_items}},
                include=["embeddings", "metadatas"]
            )
        except Exception as e:
            rospy.logerr(f"Error querying area4_collection: {e}")
            return

        ids = [md.get("name") for md in results["metadatas"]]
        num_results = len(ids)
        rospy.loginfo(f"Query returned {num_results} images.")

        if num_results > 0:
            try:
                collection.add(
                    embeddings=results["embeddings"],
                    metadatas=results["metadatas"],
                    ids=ids
                )
                rospy.loginfo(f"Stored {num_results} images in collection '{collection.name}'.")
                self.retrieval_complete_pub.publish(String(f"Query processed: {query_string}"))
            except Exception as e:
                rospy.logerr(f"Error adding queried images: {e}")
        else:
            rospy.loginfo("No images matched the query.")
            self.retrieval_complete_pub.publish(String(f"Query returned no results: {query_string}"))

    def navigation_query_callback(self, msg):
        """
        Callback for navigation image query command.
        If the command is 'query_navigation_image', sets a flag so that the next incoming image is processed.
        """
        with self.flag_lock:
            if msg.data == "query_navigation_image":
                rospy.loginfo("Received 'query_navigation_image' command. Starting image processing...")
                self.process_image_flag = True

    def image_callback(self, msg):
        """
        Callback for processing a camera image.
        Only processes the image if the process_image_flag is set.
        """
        if rospy.is_shutdown():
            return

        with self.flag_lock:
            if not self.process_image_flag:
                return
            # Reset the flag once the image is picked up for processing
            self.process_image_flag = False

        try:
            # Convert ROS Image message to a NumPy array then to a PIL image
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            pil_image = PILImage.fromarray(np_image, 'RGB')
            # Retrieve the top similar image from the active retrieved images collection
            self.retrieve_top_similar_image(pil_image)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def embed_current_observation(self, pil_image):
        """
        Embeds the current observation using the navigation image preprocessing.
        """
        img_tensor = self.preprocess_nav(pil_image).unsqueeze(0).to(self.device)
        with torch.no_grad():
            embedding = self.model.encode_image(img_tensor).cpu().numpy().flatten().tolist()
        return embedding

    def retrieve_top_similar_image(self, pil_image):
        """
        Retrieves the top similar image from the active retrieved images collection.
        """
        if self.active_retrieved_collection is None:
            rospy.logwarn("No active retrieved images collection available.")
            return

        rospy.loginfo("Embedding current observation image...")
        current_embedding = self.embed_current_observation(pil_image)

        rospy.loginfo("Retrieving embeddings from the active retrieved images collection...")
        try:
            retrieved_data = self.active_retrieved_collection.get(include=["metadatas", "embeddings"])
        except Exception as e:
            rospy.logerr(f"Error retrieving embeddings from the collection: {e}")
            return

        if len(retrieved_data['embeddings']) == 0:
            rospy.logwarn("No embeddings found in the active retrieved images collection.")
            return

        embeddings_matrix = np.array(retrieved_data['embeddings'])
        current_embedding_np = np.array(current_embedding)
        if np.linalg.norm(current_embedding_np) == 0:
            rospy.logwarn("Current observation embedding norm is zero.")
            return

        # Compute cosine similarities
        norms = np.linalg.norm(embeddings_matrix, axis=1)
        norms[norms == 0] = 1e-10  # Avoid division by zero
        similarity_scores = np.dot(embeddings_matrix, current_embedding_np) / (norms * np.linalg.norm(current_embedding_np))
        top_index = np.argmax(similarity_scores)
        top_similarity = similarity_scores[top_index]
        top_image_metadata = retrieved_data['metadatas'][top_index]

        image_id = top_image_metadata.get('name', 'Unknown')
        image_class = top_image_metadata.get('class', 'Unknown')
        rospy.loginfo(f"Top similarity: {top_similarity:.4f}")
        rospy.loginfo(f"Top image class: {image_class}, id: {image_id}")
        self.publish_similarity(top_similarity, image_class, image_id)

    def publish_similarity(self, similarity_score, image_class, image_id):
        """
        Publishes the similarity score along with the image class and id.
        """
        message = f"Similarity score: {similarity_score:.4f}, floor: {image_class}, location: {image_id}"
        rospy.loginfo(f"Publishing similarity: {message}")
        rospy.sleep(2)
        self.similarity_pub.publish(String(message))
        rospy.sleep(2)

if __name__ == "__main__":
    node = CombinedNode()
    rospy.spin()

import rospy
from std_msgs.msg import String
import torch
import open_clip
from PIL import Image
import chromadb
import json
from threading import Timer

class ImageEmbeddingNode:
    def __init__(self):
        rospy.init_node('image_embedding_node')
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        
        # Load CLIP model from Hugging Face using OpenCLIP
        model_name = 'ViT-B-32'
        pretrained_weights = 'laion2b_s34b_b79k'
        self.model, _, preprocess_val = open_clip.create_model_and_transforms(model_name, pretrained=pretrained_weights)
        self.model = self.model.to(self.device)
        self.preprocess = preprocess_val
        
        # Load the master vector DB ("area4_collection") from the persistent path for area images
        self.all_images_collection = self.load_existing_collection(
            collection_name="area4_collection", 
            persist_directory="/home/ubuntu/catkin_ws/src/scripts/area4_db"
        )
        
        # The retrieved images collection (for query results) will be stored in the same way as your provided script.
        # Its persist directory and collection name are fixed.
        self.collection_name = "retrieved_images_collection"
        self.retrieved_db_path = "/home/ubuntu/catkin_ws/src/scripts/retrieved_images_db"
        self.retrieved_images_collection = None  # Will be created/reset when a query is processed.
        
        # Publishers for status messages
        self.retrieval_complete_pub = rospy.Publisher('/image_retrieval_success', String, queue_size=10)
        self.db_reset_complete_pub = rospy.Publisher('/navigation_vector_db_reset_complete', String, queue_size=10)
        
        # Subscribers for incoming image metadata and query strings.
        rospy.Subscriber('retrieved_image_metadata', String, self.image_metadata_callback)
        rospy.Subscriber('chatgpt_output_image_query', String, self.query_callback)
        rospy.Subscriber('/reset_collection', String, self.reset_collection_callback)
        rospy.Subscriber('/image_reset', String, self.image_reset_callback)
        
        self.processed_images = 0
        self.completion_timer = None
        self.timeout_duration = rospy.Duration(1)  # 1-second timeout

    def load_existing_collection(self, collection_name, persist_directory):
        """Loads an existing collection or creates one if not found."""
        client = chromadb.PersistentClient(path=persist_directory)
        try:
            collection = client.get_collection(name=collection_name)
            rospy.loginfo(f"Loaded existing collection '{collection_name}'.")
        except Exception as e:
            rospy.loginfo(f"Collection '{collection_name}' not found, creating new one. Error: {e}")
            collection = client.create_collection(collection_name)
        return collection

    def create_or_reset_retrieved_images_collection(self, collection_name="retrieved_images_collection"):
        """
        Deletes the existing collection (if any) and creates a new one.
        This method mimics your provided script by using the persist directory:
        "/home/ubuntu/catkin_ws/src/scripts/retrieved_images_db"
        """
        persist_directory = self.retrieved_db_path
        client = chromadb.PersistentClient(path=persist_directory)
        try:
            client.delete_collection(collection_name)
            rospy.loginfo(f"Existing collection '{collection_name}' deleted.")
        except Exception as e:
            rospy.loginfo(f"No existing collection named '{collection_name}' found: {e}")
        collection = client.create_collection(collection_name)
        rospy.loginfo(f"Collection '{collection_name}' created successfully.")
        return collection

    def image_metadata_callback(self, msg):
        """Callback to process new image metadata and add it to the master DB."""
        try:
            image_metadata = json.loads(msg.data)
            self.embed_and_store_retrieved_image(image_metadata)
        except Exception as e:
            rospy.logerr(f"Error processing image metadata: {e}")

    def embed_and_store_retrieved_image(self, image_metadata):
        """
        Embeds the image using the CLIP model and stores it in the master DB ("area4_collection")
        with metadata formatted as in your provided script.
        """
        image_path = image_metadata['metadata']['path']
        class_name = image_metadata['metadata']['class']
        image_id = image_metadata['id']
        
        try:
            image = self.preprocess(Image.open(image_path)).unsqueeze(0).to(self.device)
            with torch.no_grad():
                embedding = self.model.encode_image(image).cpu().numpy().flatten().tolist()
            
            self.all_images_collection.add(
                embeddings=[embedding],
                metadatas=[{"class": class_name, "path": image_path, "name": image_id}],
                ids=[image_id]
            )
            rospy.loginfo(f"Embedded and stored image {image_id} in area4_collection.")
            self.processed_images += 1
            self.reset_completion_timer()
        except Exception as e:
            rospy.logerr(f"Error embedding image {image_id}: {e}")

    def query_callback(self, msg):
        """
        Callback to process a query string that filters images from the master DB.
        Expected query format:
        "First_floor|HallwayA_west, First_floor|HallwayA_north, First_floor|RoomA_north"
        """
        query_string = msg.data
        rospy.loginfo(f"Received query: {query_string}")
        self.query_and_store_results(query_string)

    def query_and_store_results(self, query_string):
        """
        Splits the query string and filters the master DB for images whose metadata "class"
        matches any of the query terms. The results are stored in the retrieved images collection
        using the same storage format as in your provided script.
        """
        # Split the query string on commas, then split on the pipe symbol and trim spaces.
        query_items = query_string.split(",")
        query_terms = []
        for item in query_items:
            parts = item.strip().split("|")
            for part in parts:
                term = part.strip()
                if term:
                    query_terms.append(term)
        rospy.loginfo(f"Parsed query terms: {query_terms}")

        try:
            results = self.all_images_collection.get(
                where={"class": {"$in": query_terms}},
                include=["embeddings", "metadatas"]
            )
        except Exception as e:
            rospy.logerr(f"Error querying area4_collection: {e}")
            return

        # Reconstruct ids from the metadata (assuming the "name" field stores the image id)
        ids = [md.get("name") for md in results["metadatas"]]
        num_results = len(ids)
        rospy.loginfo(f"Query returned {num_results} images.")

        # Create or reset the retrieved images collection using the same persist directory as the provided script.
        self.retrieved_images_collection = self.create_or_reset_retrieved_images_collection(self.collection_name)

        if num_results > 0:
            try:
                self.retrieved_images_collection.add(
                    embeddings=results["embeddings"],
                    metadatas=results["metadatas"],
                    ids=ids
                )
                rospy.loginfo(f"Stored {num_results} images in {self.collection_name}.")
                self.retrieval_complete_pub.publish("Query processed: " + query_string)
            except Exception as e:
                rospy.logerr(f"Error adding queried images: {e}")
        else:
            rospy.loginfo("No images matched the query.")
            self.retrieval_complete_pub.publish("Query returned no results: " + query_string)

    def reset_collection_callback(self, msg):
        """Callback to reset the master DB ("area4_collection") if requested."""
        rospy.loginfo("Resetting the master vector DB (area4_collection) based on received message.")
        self.all_images_collection = self.load_existing_collection(
            collection_name="area4_collection", 
            persist_directory="/home/ubuntu/catkin_ws/src/scripts/area4_db"
        )
        self.processed_images = 0
        rospy.sleep(3)
        self.db_reset_complete_pub.publish("Navigation vector DB reset is complete")
        rospy.loginfo("area4_collection has been reset. Published reset complete message.")

    def image_reset_callback(self, msg):
        """Callback to reset the master DB ("area4_collection") upon receiving a specific reset command."""
        if msg.data == "image reset":
            rospy.loginfo("Received 'image reset' message. Resetting area4_collection.")
            self.all_images_collection = self.load_existing_collection(
                collection_name="area4_collection", 
                persist_directory="/home/ubuntu/catkin_ws/src/scripts/area4_db"
            )
            self.processed_images = 0
            rospy.loginfo("area4_collection has been reset due to 'image reset' command.")
            self.db_reset_complete_pub.publish("Image collection reset complete")

    def reset_completion_timer(self):
        """Resets the timer that publishes a completion message after inactivity."""
        if self.completion_timer:
            self.completion_timer.shutdown()
        self.completion_timer = rospy.Timer(self.timeout_duration, self.publish_completion, oneshot=True)

    def publish_completion(self, event):
        rospy.sleep(3)
        self.retrieval_complete_pub.publish("Navigation image retrieval is complete")
        rospy.loginfo(f"All images processed. Total images: {self.processed_images}. Published completion message.")

if __name__ == "__main__":
    node = ImageEmbeddingNode()
    rospy.spin()

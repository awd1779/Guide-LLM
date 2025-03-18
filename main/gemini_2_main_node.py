#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from google import genai
import time

# Configure Gemini API
genai.configure(api_key="YOUR_GEMINI_API_KEY")

class AssistantsNode:
    def __init__(self):
        rospy.init_node('assistants_node', anonymous=True)

        # A unified callback function for all subscribers
        self.subscriber_map = {
            'assistants_input': rospy.Subscriber('assistants_input', String, self.unified_callback, callback_args='assistants_input', queue_size=10),
            'similarity_node': rospy.Subscriber('similarity_node', String, self.unified_callback, callback_args='similarity_node', queue_size=10),
            'localization_node': rospy.Subscriber('localization_node', String, self.unified_callback, callback_args='localization_node', queue_size=10),
            '/movement_executed': rospy.Subscriber('/movement_executed', String, self.unified_callback, callback_args='/movement_executed', queue_size=10),
            '/image_retrieval_success': rospy.Subscriber('/image_retrieval_success', String, self.unified_callback, callback_args='/image_retrieval_success', queue_size=10),
            '/route_calculation_subscriber': rospy.Subscriber('/route_calculation_subscriber', String, self.unified_callback, callback_args='/route_calculation_subscriber', queue_size=10),
            '/navigation_vector_db_reset_complete': rospy.Subscriber('/navigation_vector_db_reset_complete', String, self.unified_callback, callback_args='/navigation_vector_db_reset_complete', queue_size=10)
        }

        self.command_publisher = rospy.Publisher('gemini_output_commands', String, queue_size=10)
        self.response_publisher = rospy.Publisher('gemini_output_responses', String, queue_size=10)
        self.image_query_publisher = rospy.Publisher('gemini_output_image_query', String, queue_size=100)
        self.localization_query_publisher = rospy.Publisher('gemini_output_localization_query', String, queue_size=10)
        self.navigation_image_query_publisher = rospy.Publisher('gemini_output_navigation_image_query', String, queue_size=10)
        self.route_calculation_publisher = rospy.Publisher('/route_calculation_publisher', String, queue_size=10)
        self.reset_vd_publisher = rospy.Publisher('/reset_collection', String, queue_size=10)

    # Unified callback for handling all subscribed messages
    def unified_callback(self, msg, topic):
        rospy.loginfo(f"Received from {topic}: {msg.data}")
        user_input = msg.data

        response = self.get_gemini_response(user_input)
        rospy.loginfo(f"Gemini Response for {topic}: {response}")
        self.process_response(response)

    def get_gemini_response(self, prompt):
        """ Queries the Gemini 1.5 Pro API and returns a response. """
        try:
            model = genai.GenerativeModel("gemini-1.5-pro")
            response = model.generate_content(prompt)

            # Extracting response text
            if response and response.candidates:
                return response.candidates[0].content.parts[0].text
            else:
                rospy.logerr("No response received from Gemini API.")
                return "error"
        
        except Exception as e:
            rospy.logerr(f"Error in getting response: {str(e)}")
            return "error"

    def process_response(self, response):
        """ Processes and publishes responses according to predefined formats. """
        command_parts, normal_response_parts, image_query_parts, localization_query_parts, navigation_query_parts, routes_input_parts, reset_input_parts = [], [], [], [], [], [], []

        i = 0
        while i < len(response):
            if response[i] == '<':
                end_index = response.find('>', i)
                if end_index != -1:
                    command_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break
            elif response[i] == '[':
                end_index = response.find(']', i)
                if end_index != -1:
                    normal_response_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break
            elif response[i] == '$':
                end_index = response.find('$', i + 1)
                if end_index != -1:
                    image_query_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break
            elif response[i] == '@':
                end_index = response.find('@', i + 1)
                if end_index != -1:
                    localization_query_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break
            elif response[i] == '&':
                end_index = response.find('&', i + 1)
                if end_index != -1:
                    navigation_query_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break
            elif response[i] == '%':
                end_index = response.find('%', i + 1)
                if end_index != -1:
                    routes_input_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break    
            elif response[i] == '^':
                end_index = response.find('^', i + 1)
                if end_index != -1:
                    reset_input_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break  
            else:
                i += 1

        # Publish commands and responses
        if command_parts:
            self.command_publisher.publish(String(data=', '.join(command_parts)))
        if normal_response_parts:
            self.response_publisher.publish(String(data=' '.join(normal_response_parts)))
        if image_query_parts:
            self.image_query_publisher.publish(String(data=', '.join(image_query_parts)))
        if localization_query_parts:
            self.localization_query_publisher.publish(String(data=' '.join(localization_query_parts)))
        if navigation_query_parts:
            self.navigation_image_query_publisher.publish(String(data=' '.join(navigation_query_parts)))
        if routes_input_parts:
            self.route_calculation_publisher.publish(String(data=' '.join(routes_input_parts)))
        if reset_input_parts:
            self.reset_vd_publisher.publish(String(data=' '.join(reset_input_parts)))

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting Gemini Assistant Node...")
        AssistantsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")

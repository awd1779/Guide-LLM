#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from openai import OpenAI  # Import OpenAI

client = OpenAI(api_key='')

class AssistantsNode:
    def __init__(self, assistant_id):
        self.assistant_id = assistant_id
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

        self.command_publisher = rospy.Publisher('chatgpt_output_commands', String, queue_size=10)
        self.response_publisher = rospy.Publisher('chatgpt_output_responses', String, queue_size=10)
        self.curly_command_publisher = rospy.Publisher('chatgpt_output_image_query', String, queue_size=100)
        self.current_location_publisher = rospy.Publisher('chatgpt_output_localization_query', String, queue_size=10)
        self.navigation_image_query_publisher = rospy.Publisher('chatgpt_output_navigation_image_query', String, queue_size=10)
        self.route_calculation_publisher = rospy.Publisher('/route_calculation_publisher', String, queue_size=10)
        self.reset_vd_publisher = rospy.Publisher('/reset_collection', String, queue_size=10)

        self.thread_id = self.create_thread()

    def create_thread(self):
        try:
            response = client.beta.threads.create()
            rospy.loginfo(f"Thread created with ID: {response.id}")
            return response.id
        except Exception as e:
            rospy.logerr(f"Error creating thread: {str(e)}")
            return None

    # Unified callback for handling all subscribed messages
    def unified_callback(self, msg, topic):
        rospy.loginfo(f"Received from {topic}: {msg.data}")
        user_input = msg.data
        
        # Handle different topics with specific behavior if needed
        if topic == 'assistants_input':
            # For assistants_input
            response = self.get_chatgpt_response(user_input)
            rospy.loginfo(f"Assistant Response for {topic}: {response}")
            self.process_response(response)
        
        elif topic == 'similarity_node':
            # For similarity node input
            response = self.get_chatgpt_response(user_input)
            rospy.loginfo(f"Assistant Response for {topic}: {response}")
            self.process_response(response)
        
        elif topic == 'localization_node':
            # For localization node input
            response = self.get_chatgpt_response(user_input)
            rospy.loginfo(f"Assistant Response for {topic}: {response}")
            self.process_response(response)
        
        # Continue adding specific logic for other topics as necessary
        elif topic == '/movement_executed':
            # Turtlebot movement complete message
            response = self.get_chatgpt_response(user_input)
            rospy.loginfo(f"Assistant Response for {topic}: {response}")
            self.process_response(response)
        
        elif topic == '/image_retrieval_success':
            # Image retrieval success callback
            response = self.get_chatgpt_response(user_input)
            rospy.loginfo(f"Assistant Response for {topic}: {response}")
            self.process_response(response)
        
        elif topic == '/route_calculation_subscriber':
            # Route calculation input
            response = self.get_chatgpt_response(user_input)
            rospy.loginfo(f"Assistant Response for {topic}: {response}")
            self.process_response(response)
            
        elif topic == '/navigation_vector_db_reset_complete':             
            response = self.get_chatgpt_response(user_input)
            rospy.loginfo(f"Assistant Response for {topic}: {response}")
            self.process_response(response)

    def get_chatgpt_response(self, prompt):
        # Existing logic for getting a response from ChatGPT remains unchanged
        try:
            message = client.beta.threads.messages.create(
                thread_id=self.thread_id,
                role="user",
                content=prompt
            )

            run = client.beta.threads.runs.create(
                thread_id=self.thread_id,
                assistant_id=self.assistant_id
            )

            while run.status in ["queued", "in_progress"]:
                run = client.beta.threads.runs.retrieve(
                    thread_id=self.thread_id,
                    run_id=run.id
                )
                rospy.sleep(0)

            messages = client.beta.threads.messages.list(
                thread_id=self.thread_id,
                order="asc",
                after=message.id
            )

            if not messages.data:
                rospy.logerr("No messages received in response.")
                return "error"

            if not messages.data[-1].content or not messages.data[-1].content[0].text:
                rospy.logerr("Message content is not in the expected format.")
                return "error"

            response_message = messages.data[-1].content[0].text.value
            return response_message

        except Exception as e:
            rospy.logerr(f"Error in getting response: {str(e)}")
            return "error"

    def process_response(self, response):
        # The same processing logic for responses
        command_parts, normal_response_parts, curly_command_parts, current_location_parts, navigation_command_parts, routes_input_parts, reset_input_parts = [], [], [], [], [], [], []

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
                    curly_command_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break
            elif response[i] == '@':
                end_index = response.find('@', i + 1)
                if end_index != -1:
                    current_location_parts.append(response[i + 1:end_index].strip())
                    i = end_index + 1
                else:
                    break
            elif response[i] == '&':
                end_index = response.find('&', i + 1)
                if end_index != -1:
                    navigation_command_parts.append(response[i + 1:end_index].strip())
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

        # Publish commands and responses as before
        if command_parts:
            command_data = ', '.join(command_parts)
            #rospy.loginfo(f"Publishing command: {command_data}")
            self.command_publisher.publish(String(data=command_data))

        if normal_response_parts:
            normal_response_data = ' '.join(normal_response_parts)
            #rospy.loginfo(f"Publishing normal response: {normal_response_data}")
            self.response_publisher.publish(String(data=normal_response_data))
            
        if navigation_command_parts:
            current_navigation_location = ' '.join(navigation_command_parts)
            #rospy.loginfo(f"Publishing navigation_image command: {current_navigation_location}")
            self.navigation_image_query_publisher.publish(String(data=current_navigation_location))         

        if curly_command_parts:
            curly_command_data = ', '.join(curly_command_parts)
            #rospy.loginfo(f"Publishing image query command: {curly_command_data}")
            self.curly_command_publisher.publish(String(data=curly_command_data))
        
        if current_location_parts:
            current_location = ' '.join(current_location_parts)
            #rospy.loginfo(f"Publishing localization command: {current_location}")
            self.current_location_publisher.publish(String(data=current_location)) 
        
        if routes_input_parts:
            routes_input_parts_is = ' '.join(routes_input_parts)
            #rospy.loginfo(f"Publishing route retrieval command: {routes_input_parts_is}")
            self.route_calculation_publisher.publish(String(data=routes_input_parts_is)) 
        
        if reset_input_parts:
            reset_input_parts_is = ' '.join(reset_input_parts)
            #rospy.loginfo(f"Publishing resent vd  command: {reset_input_parts_is}")
            self.reset_vd_publisher.publish(String(data=reset_input_parts_is)) 

if __name__ == '__main__':
    try:
        assistant_id = 'asst_2UC2J4KUlUZsMyRurtkVQIiF'
        my_assistant = client.beta.assistants.retrieve(assistant_id)
        rospy.loginfo(f"Using Assistant: {my_assistant}")
        AssistantsNode(assistant_id)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")

#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
from groq import Groq  # Import the Groq client
import time  # Make sure to import time at the top

# 1. Initialize the Groq client using the API key from environment variables
client = Groq(api_key='')

# 2. This is our system prompt in JSON format
system_prompt = """
{
  "overview":[ "You are an agent interacting with navigation modules to guide a robot to specified locations.",
   "Every command you produce will be entered into a module that outputs results." ,
   "Do not include any unnecessary text." ,
   "Follow each steps in sequence and do not skip any steps", 
   "issuing only one command per turn and using the exact command formats provided."]
  "steps": [
    {
      "step_number": 1,
      "title": "Determine Current Location",
      "instructions": [
        "Use the command @current_location@ to retrieve the robot’s current position and heading.",
        "Proceed to Step 2."
      ]
    },
    {
      "step_number": 2,
      "title": "Route Query",
      "instructions": [
        "After detemining current location from step 1, use the route query command to request a navigation route.",
        "Command format: %Location_heading Destination%",
        "Review the returned route, noting key turns and distances, then proceed to Step 3."
      ]
    },
{
  "step_number": "3",
  "title": "Navigation Plan & Image Retrieval",
  "instructions": [
    "Generate a complete navigation plan based on the route provided by the path planning module.",
    "For each navigation step, specify the following:",
    " Start location and current heading",
    " Destination location and required heading",
    " The action command (turn or move), including direction and degrees/distance.
    " Produce movement command without <> at this step as it might triger robot movement",
    " The resulting new location and new heading after the command",
    "Include the image retrieval command for each step using the format: $First_floor|Location_heading$.",
  ],
  "example_format": {
    "navigation_step": "[Step Number]",
    "from": "[Location]",
    "to": "[Location]",
    "command": "[Turn/Move] [Direction] [Degrees/Distance]",
    "new_location": "[Location]",
    "new_heading": "[Heading]",
    "image_command": "$First_floor|[Location]_[Heading]$"
  },  
},
    {
      "step_number": 4,
      "title": "Movement Execution",
      "instructions": [
        "Wait for the responsesfrom image retrieval moudle before issuing any movement commands.",
        "Movement commands must be enclosed in angle brackets <>.",
        "Example: <forward 6.5> or <turn left 90>.",
        "Issue one movement command to progress to the next navigation step.",
        "After sending each movement command, proceed to Step 5 to verify the robot’s location."
      ]
    },
    {
      "step_number": 5,
      "title": "Position Verification",
      "instructions": [
        "After each movement, use the command &query_navigation_image& to confirm the robot’s new position.",
        "Compare the retrieved position with the expected position from the navigation plan.",
        "If the positions match, return to Step 5 to issue the next movement command."
      ]
    },
    {
      "step_number": 6,
      "title": "Localization Error Handling",
      "instructions": [
        "If the robot appears stuck or lost (for example, if the similarity score is below 0.9), restart the process from Step 1."
      ]
    }
  ]
}
"""

class AssistantsNode:
    def __init__(self, model_name):
        self.model_name = model_name
        rospy.init_node('assistants_node', anonymous=True)

        # 3. Maintain a conversation buffer (list of messages)
        self.conversation = []
        # Add the system prompt once at the beginning:
        self.conversation.append({"role": "system", "content": system_prompt})

        # Subscribers for different topics
        self.subscriber_map = {
            'assistants_input': rospy.Subscriber('assistants_input', String, self.unified_callback, callback_args='assistants_input', queue_size=10),
            'similarity_node': rospy.Subscriber('similarity_node', String, self.unified_callback, callback_args='similarity_node', queue_size=10),
            'localization_node': rospy.Subscriber('localization_node', String, self.unified_callback, callback_args='localization_node', queue_size=10),
            '/movement_executed': rospy.Subscriber('/movement_executed', String, self.unified_callback, callback_args='/movement_executed', queue_size=10),
            '/image_retrieval_success': rospy.Subscriber('/image_retrieval_success', String, self.unified_callback, callback_args='/image_retrieval_success', queue_size=10),
            '/route_calculation_subscriber': rospy.Subscriber('/route_calculation_subscriber', String, self.unified_callback, callback_args='/route_calculation_subscriber', queue_size=10),
            '/navigation_vector_db_reset_complete': rospy.Subscriber('/navigation_vector_db_reset_complete', String, self.unified_callback, callback_args='/navigation_vector_db_reset_complete', queue_size=10)
        }

        # Publishers for output topics
        self.command_publisher = rospy.Publisher('chatgpt_output_commands', String, queue_size=10)
        self.response_publisher = rospy.Publisher('chatgpt_output_responses', String, queue_size=10)
        self.curly_command_publisher = rospy.Publisher('chatgpt_output_image_query', String, queue_size=100)
        self.current_location_publisher = rospy.Publisher('chatgpt_output_localization_query', String, queue_size=10)
        self.navigation_image_query_publisher = rospy.Publisher('chatgpt_output_navigation_image_query', String, queue_size=10)
        self.route_calculation_publisher = rospy.Publisher('/route_calculation_publisher', String, queue_size=10)
        self.reset_vd_publisher = rospy.Publisher('/reset_collection', String, queue_size=10)

    def unified_callback(self, msg, topic):
        """
        Callback for all subscribed topics. Each time the user or system
        provides new input, we use our LLM to get a response and process it.
        """
        rospy.loginfo(f"Received from {topic}: {msg.data}")
        user_input = msg.data

        # Get the LLM's response using our conversation buffer
        response = self.get_groq_response(user_input)
        rospy.loginfo(f"Assistant Response for {topic}: {response}")
        self.process_response(response)

    def get_groq_response(self, user_input):
        """
        Uses the Groq chat completion endpoint with the conversation buffer.
        1) We append the user's message to the conversation.
        2) We send the entire conversation to the LLM.
        3) We append the LLM's response to the conversation.
        """
        try:
            #self.conversation.append({"role": "system", "content": system_prompt})
            # Append new user message
            self.conversation.append({"role": "user", "content": user_input})
            time.sleep(5)

            # Query the LLM with the entire conversation
            response = client.chat.completions.create(
                model=self.model_name,
                messages=self.conversation,
                stream=False,
                temperature=0.2,
                top_p=1
            )

            # Extract the assistant's text
            assistant_message = response.choices[0].message.content

            # Append the assistant message
            self.conversation.append({"role": "assistant", "content": assistant_message})

            return assistant_message
        except Exception as e:
            rospy.logerr(f"Error in getting response: {str(e)}")
            return "error"

    def process_response(self, response):
        """
        Processes the assistant's response, looking for special markup:
          - <...> : Movement or other commands
          - [...] : Normal textual responses
          - $...$ : Image queries
          - @...@ : Localization queries
          - &...& : Navigation images or checks
          - %...% : Route calculation
          - ^...^ : Reset commands
        Publishes each part on the appropriate ROS topics.
        """
        command_parts, normal_response_parts = [], []
        curly_command_parts, current_location_parts = [], []
        navigation_command_parts, routes_input_parts = [], []
        reset_input_parts = []

        i = 0
        while i < len(response):
            char = response[i]

            if char == '<':
                end_index = response.find('>', i)
                if end_index != -1:
                    command = response[i+1:end_index].strip()
                    command_parts.append(command)
                    i = end_index + 1
                else:
                    i += 1

            elif char == '[':
                end_index = response.find(']', i)
                if end_index != -1:
                    text = response[i+1:end_index].strip()
                    normal_response_parts.append(text)
                    i = end_index + 1
                else:
                    i += 1

            elif char == '$':
                end_index = response.find('$', i+1)
                if end_index != -1:
                    image_query = response[i+1:end_index].strip()
                    curly_command_parts.append(image_query)
                    i = end_index + 1
                else:
                    i += 1

            elif char == '@':
                end_index = response.find('@', i+1)
                if end_index != -1:
                    loc_query = response[i+1:end_index].strip()
                    current_location_parts.append(loc_query)
                    i = end_index + 1
                else:
                    i += 1

            elif char == '&':
                end_index = response.find('&', i+1)
                if end_index != -1:
                    nav_command = response[i+1:end_index].strip()
                    navigation_command_parts.append(nav_command)
                    i = end_index + 1
                else:
                    i += 1

            elif char == '%':
                end_index = response.find('%', i+1)
                if end_index != -1:
                    route_command = response[i+1:end_index].strip()
                    routes_input_parts.append(route_command)
                    i = end_index + 1
                else:
                    i += 1

            elif char == '^':
                end_index = response.find('^', i+1)
                if end_index != -1:
                    reset_command = response[i+1:end_index].strip()
                    reset_input_parts.append(reset_command)
                    i = end_index + 1
                else:
                    i += 1

            else:
                i += 1

        # Publish each part on the correct topic
        if command_parts:
            self.command_publisher.publish(String(data=', '.join(command_parts)))
        if normal_response_parts:
            self.response_publisher.publish(String(data=' '.join(normal_response_parts)))
        if curly_command_parts:
            self.curly_command_publisher.publish(String(data=', '.join(curly_command_parts)))
        if current_location_parts:
            self.current_location_publisher.publish(String(data=' '.join(current_location_parts)))
        if navigation_command_parts:
            self.navigation_image_query_publisher.publish(String(data=' '.join(navigation_command_parts)))
        if routes_input_parts:
            self.route_calculation_publisher.publish(String(data=' '.join(routes_input_parts)))
        if reset_input_parts:
            self.reset_vd_publisher.publish(String(data=' '.join(reset_input_parts)))

if __name__ == '__main__':
    try:
        # Specify the Groq model name (e.g. "groq-llm") as per your configuration
        model_name = 'llama-3.2-90b-vision-preview'
        AssistantsNode(model_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")

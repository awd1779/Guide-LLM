#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String
import anthropic  # Import Claude API
import json
import os

# Initialize Claude client with your API key
client = anthropic.Anthropic(
    api_key=""
)

# System prompt as JSON. Note: Commas have been added to create valid JSON.
system_prompt = """
{
  "overview": [
    "You are an agent interacting with navigation modules to guide a robot to specified locations.",
    "Every command you produce will be entered into a module that outputs results.",
    "Do not include any unnecessary text.",
    "Follow each step in sequence and do not skip any steps.",
    "Issue only one command per turn and use the exact command formats provided."
  ],
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
        "After determining current location from step 1, use the route query command to request a navigation route.",
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
        "Start location and current heading",
        "Destination location and required heading",
        "The action command (turn or move), including direction and degrees/distance.",
        "Produce movement command without <> at this step as it might trigger robot movement.",
        "The resulting new location and new heading after the command.",
        "Include the image retrieval command for each step using the format: $First_floor|Location_heading$."
      ],
      "example_format": {
        "navigation_step": "[Step Number]",
        "from": "[Location]",
        "to": "[Location]",
        "command": "[Turn/Move] [Direction] [Degrees/Distance]",
        "new_location": "[Location]",
        "new_heading": "[Heading]",
        "image_command": "$First_floor|[Location]_[Heading]$"
      }
    },
    {
      "step_number": 4,
      "title": "Movement Execution",
      "instructions": [
        "Wait for the responses from the image retrieval module before issuing any movement commands.",
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

# File used to temporarily persist conversation history during a session.
HISTORY_FILE = "conversation_history.json"

class AssistantsNode:
    def __init__(self):
        rospy.init_node('assistants_node', anonymous=True)
        
        # Remove any existing conversation history file at startup
        if os.path.exists(HISTORY_FILE):
            try:
                os.remove(HISTORY_FILE)
                rospy.loginfo("Previous conversation history deleted.")
            except Exception as e:
                rospy.logerr(f"Error deleting history file: {str(e)}")
        
        # Start with an empty conversation history for every session.
        self.conversation_history = []

        # Subscribers for various topics
        self.subscriber_map = {
            'assistants_input': rospy.Subscriber('assistants_input', String, self.unified_callback, callback_args='assistants_input', queue_size=10),
            'similarity_node': rospy.Subscriber('similarity_node', String, self.unified_callback, callback_args='similarity_node', queue_size=10),
            'localization_node': rospy.Subscriber('localization_node', String, self.unified_callback, callback_args='localization_node', queue_size=10),
            '/movement_executed': rospy.Subscriber('/movement_executed', String, self.unified_callback, callback_args='/movement_executed', queue_size=10),
            '/image_retrieval_success': rospy.Subscriber('/image_retrieval_success', String, self.unified_callback, callback_args='/image_retrieval_success', queue_size=10),
            '/route_calculation_subscriber': rospy.Subscriber('/route_calculation_subscriber', String, self.unified_callback, callback_args='/route_calculation_subscriber', queue_size=10),
            '/navigation_vector_db_reset_complete': rospy.Subscriber('/navigation_vector_db_reset_complete', String, self.unified_callback, callback_args='/navigation_vector_db_reset_complete', queue_size=10)
        }

        # Publishers for output commands and responses
        self.command_publisher = rospy.Publisher('chatgpt_output_commands', String, queue_size=10)
        self.response_publisher = rospy.Publisher('chatgpt_output_responses', String, queue_size=10)
        self.curly_command_publisher = rospy.Publisher('chatgpt_output_image_query', String, queue_size=100)
        self.current_location_publisher = rospy.Publisher('chatgpt_output_localization_query', String, queue_size=10)
        self.navigation_image_query_publisher = rospy.Publisher('chatgpt_output_navigation_image_query', String, queue_size=10)
        self.route_calculation_publisher = rospy.Publisher('/route_calculation_publisher', String, queue_size=10)
        self.reset_vd_publisher = rospy.Publisher('/reset_collection', String, queue_size=10)

        # Ensure conversation history is cleared on shutdown (optional, since it's not loaded on restart)
        rospy.on_shutdown(self.clear_history)

    def clear_history(self):
        """Clear the conversation history file if it exists."""
        if os.path.exists(HISTORY_FILE):
            try:
                os.remove(HISTORY_FILE)
                rospy.loginfo("Conversation history cleared on shutdown.")
            except Exception as e:
                rospy.logerr(f"Error clearing history file on shutdown: {str(e)}")
    
    def save_history(self):
        """Save current conversation history to a file during the session (optional)."""
        try:
            with open(HISTORY_FILE, 'w') as f:
                json.dump(self.conversation_history, f)
        except Exception as e:
            rospy.logerr(f"Error saving history: {str(e)}")

    def unified_callback(self, msg, topic):
        rospy.loginfo(f"Received from {topic}: {msg.data}")
        user_input = msg.data
        
        # Get response from Claude (using the current session's conversation history)
        response = self.get_claude_response(user_input)
        rospy.loginfo(f"Claude Response for {topic}: {response}")
        self.process_response(response)

    def get_claude_response(self, prompt):
        """Send prompt to Claude including session conversation history and return the response."""
        # Append the new user message to conversation history
        self.conversation_history.append({"role": "user", "content": prompt})
        self.save_history()  # Save during session if needed

        try:
            response = client.messages.create(
                model="claude-3-5-sonnet-20241022",  # Use a valid Claude chat model
                max_tokens=1024,
                temperature=1,
                system=system_prompt, 
                messages=self.conversation_history
            )

            if response and response.content:
                assistant_reply = response.content[0].text
                # Append assistant's reply to conversation history and save
                self.conversation_history.append({"role": "assistant", "content": assistant_reply})
                self.save_history()
                return assistant_reply

            return "error"

        except Exception as e:
            rospy.logerr(f"Error in getting response: {str(e)}")
            return "error"

    def process_response(self, response):
        """Parse the response and publish different parts to the corresponding ROS topics."""
        command_parts = []
        normal_response_parts = []
        curly_command_parts = []
        current_location_parts = []
        navigation_command_parts = []
        routes_input_parts = []
        reset_input_parts = []

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

        # Publish parsed parts to their respective topics
        if command_parts:
            self.command_publisher.publish(String(data=', '.join(command_parts)))
        if normal_response_parts:
            self.response_publisher.publish(String(data=' '.join(normal_response_parts)))
        if navigation_command_parts:
            self.navigation_image_query_publisher.publish(String(data=' '.join(navigation_command_parts)))
        if curly_command_parts:
            self.curly_command_publisher.publish(String(data=', '.join(curly_command_parts)))
        if current_location_parts:
            self.current_location_publisher.publish(String(data=' '.join(current_location_parts)))
        if routes_input_parts:
            self.route_calculation_publisher.publish(String(data=' '.join(routes_input_parts)))
        if reset_input_parts:
            self.reset_vd_publisher.publish(String(data=' '.join(reset_input_parts)))

if __name__ == '__main__':
    try:
        AssistantsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")

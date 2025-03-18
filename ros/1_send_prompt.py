#!/usr/bin/env python3

import rospy
import sys
import select
from std_msgs.msg import String

class PromptSender:
    def __init__(self):
        self.publisher = rospy.Publisher('assistants_input', String, queue_size=10)
        rospy.init_node('prompt_sender', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz

    def send_prompt(self, prompt):
        rospy.loginfo(f"Sending prompt: {prompt}")
        self.publisher.publish(String(data=prompt))
        self.rate.sleep()

if __name__ == '__main__':
    try:
        sender = PromptSender()
        print("Enter your prompt for ChatGPT: ", end='', flush=True)  # Print prompt once
        while not rospy.is_shutdown():
            # Use select to wait for input with a 1-second timeout
            ready, _, _ = select.select([sys.stdin], [], [], 1.0)
            if ready:
                prompt = sys.stdin.readline().strip()
                if prompt:
                    sender.send_prompt(prompt)
                    print("Enter your prompt for ChatGPT: ", end='', flush=True)  # Print prompt again after input
    except rospy.ROSInterruptException:
        pass

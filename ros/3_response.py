#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ResponseHandler:
    def __init__(self):
        rospy.init_node('response_node', anonymous=True)
        self.subscriber = rospy.Subscriber('chatgpt_output_responses', String, self.response_callback)
        rospy.spin()

    def response_callback(self, msg):
        rospy.loginfo(f"Received response: {msg.data}")
        # Process the response as needed
        # For example, you might save it to a file or perform some other action

if __name__ == '__main__':
    try:
        ResponseHandler()
    except rospy.ROSInterruptException:
        pass

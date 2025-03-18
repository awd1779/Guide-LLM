#!/usr/bin/env python3

import re
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from queue import Queue
from threading import Thread, Event

class TurtlesimController:
    def __init__(self):
        rospy.init_node('turtlesim_controller', anonymous=True)

        # Subscribers and publishers
        self.subscriber = rospy.Subscriber('chatgpt_output_commands', String, self.listener_callback)
        self.command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.movement_completed_publisher = rospy.Publisher('/movement_executed', String, queue_size=10)

        self.rate = rospy.Rate(100)  # 100Hz
        self.default_linear_speed = 0.7  # m/s
        self.default_angular_speed = 0.05  # rad/s

        self.command_queue = Queue()
        self.command_event = Event()
        self.shutdown_event = Event()
        self.command_thread = Thread(target=self.process_command_queue)
        self.command_thread.start()

        self.total_distance_traveled = 0.0
        self.total_angle_turned = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.initialized = False

        # Variables for movement detection
        self.last_movement_time = None
        self.last_x = None
        self.last_y = None
        self.last_yaw = None

    def listener_callback(self, msg):
        rospy.loginfo(f"Received command: {msg.data}")
        commands = msg.data.split(',')
        for command in commands:
            self.command_queue.put(command.strip())
        self.command_event.set()

    def odom_callback(self, msg):
        if not self.initialized:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.initialized = True

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(orientation_q)

    def euler_from_quaternion(self, q):
        import tf.transformations
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def process_command_queue(self):
        while not self.shutdown_event.is_set():
            self.command_event.wait()
            if not self.command_queue.empty():
                command = self.command_queue.get()
                self.process_command(command)
                self.command_queue.task_done()
                rospy.sleep(0)  # Delay to allow the robot to settle

                # Clear the remaining commands in the queue
                with self.command_queue.mutex:
                    self.command_queue.queue.clear()

            self.command_event.clear()

    def normalize_command(self, command):
        """Normalize commands to a standard format."""
        command = command.strip().lower()
        # Replace "move forward" or "go forward" with "forward"
        command = re.sub(r'^(move|go)\s+forward', 'forward', command)
        return command

    def process_command(self, command):
        rospy.loginfo(f"Processing command: {command}")

        command = self.normalize_command(command)

        # Check for stop command
        if command == "stop":
            self.stop_robot()
            return

        # Parse the command into action and value
        parts = command.split()
        action = None
        value = None

        # Handle commands like "left", "right", "forward", "back", "reverse"
        if len(parts) == 1:
            if parts[0] in ["left", "right"]:
                action = parts[0]
                value = 90  # Default angle value
            elif parts[0] in ["forward", "back", "reverse"]:
                action = parts[0]
                value = 1.0  # Default distance value

        # Handle commands like "turn right 90"
        elif len(parts) == 3 and parts[0] == "turn":
            if parts[1] in ["left", "right"]:
                action = parts[1]
                try:
                    value = float(parts[2])
                except ValueError:
                    rospy.loginfo(f"Invalid value in command: {command}")
                    return

        # Handle commands like "forward 2", "backward 3", or "reverse 1.8"
        elif len(parts) == 2:
            if parts[0] in ["forward", "backward", "reverse"]:
                action = parts[0]
                try:
                    value = float(parts[1])
                except ValueError:
                    rospy.loginfo(f"Invalid value in command: {command}")
                    return

        if action is None or value is None:
            rospy.loginfo(f"Invalid command format: {command}")
            return

        # Map synonyms to primary actions
        action_map = {
            'forward': 'forward',
            'back': 'backward',
            'backward': 'backward',
            'reverse': 'backward',  # Map "reverse" to "backward"
            'left': 'left',
            'right': 'right'
        }

        # Normalize action
        action = action_map.get(action, None)
        if action is None:
            rospy.loginfo(f"Invalid action: {command}")
            return

        # Dictionary to map actions to functions
        action_function_map = {
            'forward': self.execute_distance_command,
            'backward': self.execute_distance_command,
            'left': self.execute_angle_command,
            'right': self.execute_angle_command
        }

        # Execute the appropriate command
        if action in action_function_map:
            action_function_map[action](action, value)
        else:
            rospy.loginfo(f"Invalid action: {action}")

    def stop_robot(self):
        rospy.loginfo("Stopping the robot")
        self.command_publisher.publish(Twist())  # Publish a zero Twist message to stop the robot
        self.movement_completed_publisher.publish("Robot stopped")  # Publish stop message

    def execute_distance_command(self, action, distance):
        speed = self.default_linear_speed
        if action == 'backward':
            speed = -speed

        start_x = self.current_x
        start_y = self.current_y
        initial_yaw = self.current_yaw

        twist = Twist()
        twist.linear.x = speed

        rospy.loginfo(f"Executing distance command: {action} {distance} meters at {speed} m/s")

        # Initialize movement detection variables
        self.last_movement_time = rospy.get_time()
        self.last_x = self.current_x
        self.last_y = self.current_y
        movement_threshold = 0.01  # Minimum movement to consider (in meters)
        time_threshold = 3.0       # Time to wait before checking for movement (in seconds)

        while not self.shutdown_event.is_set():
            current_distance = math.sqrt((self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2)
            if current_distance >= distance:
                rospy.loginfo(f"Target distance reached: {current_distance} meters")
                break

            # Check for lack of movement
            time_since_last_movement = rospy.get_time() - self.last_movement_time
            distance_moved = math.sqrt((self.current_x - self.last_x) ** 2 + (self.current_y - self.last_y) ** 2)

            if time_since_last_movement >= time_threshold:
                if distance_moved < movement_threshold:
                    rospy.loginfo("Robot is not moving")
                    rospy.loginfo(f"Distance traveled since last command: {current_distance:.2f} meters")
                    self.stop_robot()
                    # Publish to /movement_executed
                    message = f"Robot stopped: distance traveled {current_distance:.2f} meters"
                    self.movement_completed_publisher.publish(message)
                    return
                else:
                    # Update last movement time and position
                    self.last_movement_time = rospy.get_time()
                    self.last_x = self.current_x
                    self.last_y = self.current_y

            # Adjust yaw to keep moving straight
            yaw_error = self.normalize_angle(initial_yaw - self.current_yaw)
            twist.angular.z = 3 * yaw_error  # Simple proportional control

            self.command_publisher.publish(twist)
            self.rate.sleep()

        self.command_publisher.publish(Twist())  # Stop the robot
        self.total_distance_traveled += current_distance  # Update with actual distance traveled
        rospy.loginfo(f"Total distance traveled: {self.total_distance_traveled} meters")
        rospy.loginfo(f"Distance traveled since last command: {current_distance:.2f} meters")

        # Publish signal to indicate movement completion with distance traveled
        message = f"{action.capitalize()} complete: distance traveled {current_distance:.2f} meters"
        self.movement_completed_publisher.publish(message)

    def execute_angle_command(self, action, angle):
        speed = self.default_angular_speed
        if action == 'right':
            speed = -speed

        start_yaw = self.current_yaw
        target_yaw = start_yaw + math.radians(angle) if action == 'left' else start_yaw - math.radians(angle)

        # Normalize the target yaw to be within -pi to pi
        target_yaw = self.normalize_angle(target_yaw)

        twist = Twist()
        twist.angular.z = speed

        rospy.loginfo(f"Executing angle command: {action} {angle} degrees at {speed} rad/s")
        rospy.loginfo(f"Current yaw: {math.degrees(start_yaw):.2f}, Target yaw: {math.degrees(target_yaw):.2f}")

        while not self.shutdown_event.is_set():
            current_yaw = self.current_yaw
            yaw_diff = self.normalize_angle(target_yaw - current_yaw)
            angle_turned = abs(self.normalize_angle(current_yaw - start_yaw))

            rospy.loginfo(f"Current yaw: {math.degrees(current_yaw):.2f}, Target yaw: {math.degrees(target_yaw):.2f}, Yaw difference: {math.degrees(yaw_diff):.2f}")

            if abs(yaw_diff) < 0.01:  # Small threshold to account for precision issues
                rospy.loginfo(f"Target angle reached: {math.degrees(current_yaw):.2f} degrees")
                break

            self.command_publisher.publish(twist)
            self.rate.sleep()

        self.command_publisher.publish(Twist())  # Stop the robot
        self.total_angle_turned += math.degrees(angle_turned)  # Update with actual angle turned
        rospy.loginfo(f"Total angle turned: {self.total_angle_turned} degrees")
        rospy.loginfo(f"Angle turned since last command: {math.degrees(angle_turned):.2f} degrees")

        # Publish signal to indicate movement completion with angle turned
        message = f"{action.capitalize()} complete: angle turned {math.degrees(angle_turned):.2f} degrees"
        self.movement_completed_publisher.publish(message)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def shutdown(self):
        rospy.loginfo("Shutting down TurtlesimController")
        self.shutdown_event.set()
        self.command_event.set()
        self.command_thread.join()
        rospy.loginfo("TurtlesimController shutdown complete")

if __name__ == '__main__':
    try:
        controller = TurtlesimController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.shutdown()

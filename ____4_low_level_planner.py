#!/usr/bin/env python3

import rospy
import math
from queue import Queue
from threading import Thread, Event
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations

class TurtlesimController:
    def __init__(self):
        rospy.init_node('turtlesim_controller', anonymous=True)

        # Subscribers and publishers
        self.subscriber = rospy.Subscriber('chatgpt_output_commands', String, self.listener_callback)
        self.command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.movement_completed_publisher = rospy.Publisher('/movement_executed', String, queue_size=10)

        self.rate = rospy.Rate(10)  # 100Hz loop
        self.default_linear_speed = 0.6  # m/s
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

        # Movement detection variables
        self.last_movement_time = None
        self.last_x = None
        self.last_y = None

    def listener_callback(self, msg):
        for command in msg.data.split(','):
            self.command_queue.put(command.strip())
        self.command_event.set()

    def odom_callback(self, msg):
        if not self.initialized:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.initialized = True

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)[2]

    def process_command_queue(self):
        while not self.shutdown_event.is_set():
            self.command_event.wait()
            while not self.command_queue.empty():
                command = self.command_queue.get()
                self.process_command(command)
                self.command_queue.task_done()
                rospy.sleep(1)  # Brief delay between commands
            self.command_event.clear()

    def normalize_command(self, command):
        return command.strip().lower()

    def process_command(self, command):
        command = self.normalize_command(command)
        if command == "stop":
            self.stop_robot()
            return

        parts = command.split()
        action = None
        value = None

        if len(parts) == 1:
            if parts[0] in ["left", "right"]:
                action = parts[0]
                value = 90  # Default angle
            elif parts[0] in ["forward", "back", "reverse"]:
                action = parts[0]
                value = 1.0  # Default distance
        elif len(parts) == 3 and parts[0] == "turn" and parts[1] in ["left", "right"]:
            action = parts[1]
            try:
                value = float(parts[2])
            except ValueError:
                return
        elif len(parts) == 2 and parts[0] in ["forward", "backward", "reverse"]:
            action = parts[0]
            try:
                value = float(parts[1])
            except ValueError:
                return

        if action is None or value is None:
            return

        action_map = {
            'forward': 'forward',
            'back': 'backward',
            'backward': 'backward',
            'reverse': 'backward',
            'left': 'left',
            'right': 'right'
        }
        action = action_map.get(action, None)
        if action is None:
            return

        action_function_map = {
            'forward': self.execute_distance_command,
            'backward': self.execute_distance_command,
            'left': self.execute_angle_command,
            'right': self.execute_angle_command
        }
        if action in action_function_map:
            action_function_map[action](action, value)

    def stop_robot(self):
        self.command_publisher.publish(Twist())
        self.movement_completed_publisher.publish("Robot stopped")

    def execute_distance_command(self, action, distance):
        speed = self.default_linear_speed if action != 'backward' else -self.default_linear_speed
        start_x = self.current_x
        start_y = self.current_y
        initial_yaw = self.current_yaw

        twist = Twist()
        twist.linear.x = speed

        self.last_movement_time = rospy.get_time()
        self.last_x = self.current_x
        self.last_y = self.current_y
        movement_threshold = 0.01  # meters
        time_threshold = 3.0       # seconds

        while not self.shutdown_event.is_set():
            current_distance = math.sqrt((self.current_x - start_x) ** 2 +
                                         (self.current_y - start_y) ** 2)
            if current_distance >= distance:
                break

            time_since_last = rospy.get_time() - self.last_movement_time
            distance_moved = math.sqrt((self.current_x - self.last_x) ** 2 +
                                       (self.current_y - self.last_y) ** 2)
            if time_since_last >= time_threshold and distance_moved < movement_threshold:
                self.stop_robot()
                self.movement_completed_publisher.publish(
                    f"Robot stopped: distance traveled {current_distance:.2f} meters"
                )
                return
            else:
                self.last_movement_time = rospy.get_time()
                self.last_x = self.current_x
                self.last_y = self.current_y

            yaw_error = self.normalize_angle(initial_yaw - self.current_yaw)
            twist.angular.z = 0.2 * yaw_error
            self.command_publisher.publish(twist)
            self.rate.sleep()

        self.command_publisher.publish(Twist())
        self.total_distance_traveled += current_distance
        self.movement_completed_publisher.publish(
            f"{action.capitalize()} complete: distance traveled {current_distance:.2f} meters"
        )

    def execute_angle_command(self, action, angle):
        speed = self.default_angular_speed if action == 'left' else -self.default_angular_speed
        start_yaw = self.current_yaw
        target_yaw = start_yaw + math.radians(angle) if action == 'left' else start_yaw - math.radians(angle)
        target_yaw = self.normalize_angle(target_yaw)

        twist = Twist()
        twist.angular.z = speed

        while not self.shutdown_event.is_set():
            yaw_diff = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_diff) < 0.01:
                break
            self.command_publisher.publish(twist)
            self.rate.sleep()

        self.command_publisher.publish(Twist())
        angle_turned = abs(self.normalize_angle(self.current_yaw - start_yaw))
        self.total_angle_turned += math.degrees(angle_turned)
        self.movement_completed_publisher.publish(
            f"{action.capitalize()} complete: angle turned {math.degrees(angle_turned):.2f} degrees"
        )

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

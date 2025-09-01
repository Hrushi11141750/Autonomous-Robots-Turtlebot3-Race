#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from autorace_real_interfaces.action import AvoidObstacle
import cv2
import numpy as np
import threading

# Define the main node class
class AvoidObstacleReal(Node):
    def __init__(self):
        # Initialize the node with name
        super().__init__('avoid_obstacle_real_action')

        # Define QoS settings for LiDAR and camera
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscribe to LiDAR topic for obstacle detection
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)
        # Subscribe to camera topic for white line detection
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, qos_profile)

        # Publisher for sending velocity commands to the robot
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a timer to run the control loop every 0.1 seconds (10 Hz)
        self.create_timer(0.1, self.control_loop)

        # Bridge to convert ROS Image messages to OpenCV images
        self.cv_bridge = CvBridge()

        # Initialize sensor state variables
        self.dist_ahead = float('inf')       # Distance in front of robot
        self.dist_right = float('inf')       # Distance to the right of robot
        self.front_right_open = False        # Flag indicating if front-right is clear

        # Initialize control state flags
        self.in_wall_follow_mode = False     # Flag indicating wall-follow mode
        self.detected_line = False           # Flag indicating detection of white line
        self.has_turned = False              # Flag indicating 90-degree turn completed
        self.turn_counter = 0                # Counter for turn duration
        self.in_turn_sequence = False        # Flag indicating turn sequence is active

        # Initialize action server state flags
        self.active = False                  # Flag indicating if the action server is active
        self._goal_handle = None             # Stores current goal handle
        self.goal_done = False               # Flag indicating goal completion

        # Create the ActionServer with execute, goal, and cancel callbacks
        self.action_server = ActionServer(
            self,
            AvoidObstacle,
            'avoid_obstacle_real_action',    # Action name for CLI usage
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def preprocess_scan(self, ranges):
        # Replace all 0.0 values with inf for consistency across sim and real bot
        return [r if r > 0.0 else float('inf') for r in ranges]

    def lidar_callback(self, scan_msg):
        # Preprocess LiDAR data to replace 0.0 with inf
        cleaned_ranges = self.preprocess_scan(scan_msg.ranges)
        # Calculate minimum valid distance ahead
        self.dist_ahead = self.get_min_valid(cleaned_ranges[320:360] + cleaned_ranges[0:5])
        # Calculate minimum valid distance on the right
        self.dist_right = self.get_min_valid([cleaned_ranges[i] for i in range(265, 290)])
        # Calculate if front-right is open
        front_right = self.get_min_valid([cleaned_ranges[i] for i in range(30, 60)])
        self.front_right_open = front_right > 0.6

    def camera_callback(self, img_msg):
        try:
            # Convert ROS Image to OpenCV image
            image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as err:
            self.get_logger().error(f"Image conversion failed: {err}")
            return

        # Extract ROI for detecting white lines
        h, w = image.shape[:2]
        roi = image[int(0.7 * h):, int(0.35 * w):int(0.65 * w)]
        # Convert ROI to grayscale for thresholding
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        # Calculate white area fraction to detect white lines
        white_area = cv2.countNonZero(thresh) / thresh.size
        self.detected_line = white_area > 0.1 and not self.has_turned

        # Visualization for debugging on local development
        cv2.rectangle(image, (int(0.35 * w), int(0.7 * h)), (int(0.65 * w), h), (0, 255, 0), 2)
        cv2.imshow("View", image)
        cv2.imshow("ROI", roi)
        cv2.imshow("Threshold", thresh)
        cv2.waitKey(1)

    def get_min_valid(self, data):
        # Return the smallest valid LiDAR reading, ignoring inf values
        valid = [d for d in data if 0.05 < d < float('inf')]
        return min(valid) if valid else float('inf')

    def goal_callback(self, goal_request):
        # Always accept new goals from action client
        self.get_logger().info('Received goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Handle goal cancellation requests from action client
        self.get_logger().info('Received cancel request.')
        if self._goal_handle:
            self._goal_handle.canceled()
            self.active = False
            self._goal_handle = None
            self.stop_robot()
            self.goal_done = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Handle goal execution logic
        self.get_logger().info('Executing avoid obstacle action...')
        self.active = True
        self._goal_handle = goal_handle
        self.goal_done = False

        # Reset state flags
        self.in_wall_follow_mode = False
        self.detected_line = False
        self.has_turned = False
        self.turn_counter = 0
        self.in_turn_sequence = False

        # Create result object with success and message fields
        result = AvoidObstacle.Result()
        result.success = True
        

        # Use threading to monitor for goal completion while spinning
        done_event = threading.Event()

        def monitor_goal():
            while not self.goal_done and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
            else:
                goal_handle.succeed()
                self.get_logger().info('Goal succeeded.')
            done_event.set()

        # Start the monitoring thread and wait for it to finish
        thread = threading.Thread(target=monitor_goal)
        thread.start()
        done_event.wait()

        # Cleanup after goal completion
        self.active = False
        self._goal_handle = None
        self.stop_robot()

        return result

    def stop_robot(self):
        # Publishes zero velocities to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

    def control_loop(self):
        # If action not active, keep robot stopped
        if not self.active:
            self.stop_robot()
            return

        # Initialize velocity message
        twist = Twist()
        target_distance = 0.18
        stop_distance = 0.25

        # Detect white line to trigger 90-degree turn
        if self.detected_line and not self.has_turned and not self.in_turn_sequence:
            self.in_turn_sequence = True
            self.turn_counter = 0
            self.get_logger().info("White line spotted, beginning left turn")

        if self.in_turn_sequence:
            # Rotate in place
            twist.linear.x = 0.0
            twist.angular.z = 0.8
            self.turn_counter += 1
            if self.turn_counter >= 20:
                self.in_turn_sequence = False
                self.has_turned = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Turn finished")
            self.velocity_publisher.publish(twist)
            return

        if self.has_turned:
            # After turn, mark action as succeeded
            if self._goal_handle and self._goal_handle.is_active:
                self.get_logger().info('Action succeeded: Turn completed.')
                self.goal_done = True
            return

        # Wall-following and obstacle avoidance logic
        if self.dist_ahead < stop_distance or self.in_wall_follow_mode:
            self.in_wall_follow_mode = True
            if self.dist_ahead < stop_distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.35
                self.get_logger().info("Obstacle ahead — turning left")
            elif self.front_right_open:
                twist.linear.x = 0.1
                twist.angular.z = -0.34
                self.get_logger().info("Lost wall front-right — turning right")
            else:
                error = self.dist_right - target_distance
                twist.linear.x = 0.1
                twist.angular.z = -2.4 * error
                self.get_logger().info(f"Wall-following with error: {error:.2f}")
        else:
            # Clear path, move forward
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.in_wall_follow_mode = False
            self.get_logger().info("Clear path — moving forward")

        # Publish calculated velocities
        self.velocity_publisher.publish(twist)

# Entry point for ROS2 node
def main(args=None):
    rclpy.init(args=args)
    node = AvoidObstacleReal()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Import required libraries and ROS2 modules
import subprocess
import signal
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Bool
from enum import Enum
from rclpy.action import ActionClient
from autorace_real_interfaces.action import AvoidObstacle
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import sys

# Define states of the autorace state machine
class RaceState(Enum):
    WAIT_FOR_START = 0
    FOLLOW_ROAD = 1
    AVOID_OBSTACLE = 2
    FOLLOW_ROAD_AGAIN = 3
    ENTERING_TUNNEL = 4

# Main autorace node class
class AutoraceReal(Node):
    def __init__(self):
        super().__init__('autorace_real')
        self.state = RaceState.WAIT_FOR_START  # Initial state
        self.process = None  # Process handler for subprocess
        self.nav2_process = None  # Process handler for nav2
        self.get_logger().info('Autorace State Machine Initialized')

        # Create QoS profile for image and scan
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)

        # Flags and utilities
        self.obstacle_flag = False
        self.yellow_detected = False
        self.bridge = CvBridge()

        # Action client for obstacle avoidance
        self.avoid_client = ActionClient(self, AvoidObstacle, 'avoid_obstacle_real_action')
        self.waiting_for_obstacle_done = False

        # Timer for running state machine
        self.timer = self.create_timer(1.0, self.run_state_machine)

    # Core state machine logic
    def run_state_machine(self):
        if self.state == RaceState.WAIT_FOR_START:
            self.get_logger().info('Starting Lane Following...')
            self.launch_follow_road()
            self.state = RaceState.FOLLOW_ROAD

        elif self.state == RaceState.FOLLOW_ROAD:
            if self.obstacle_flag:
                self.kill_process()
                time.sleep(0.2)
                self.stop_robot()
                time.sleep(1.0)
                self.launch_avoid_obstacle()
                self.state = RaceState.AVOID_OBSTACLE
                self.obstacle_flag = False

        elif self.state == RaceState.AVOID_OBSTACLE:
            if not self.waiting_for_obstacle_done:
                self.get_logger().info('Sending AvoidObstacle goal...')
                goal_msg = AvoidObstacle.Goal()
                self.avoid_client.wait_for_server()
                self.future_goal = self.avoid_client.send_goal_async(goal_msg)
                self.future_goal.add_done_callback(self.avoid_done_callback)
                self.waiting_for_obstacle_done = True

        elif self.state == RaceState.FOLLOW_ROAD_AGAIN:
            if self.yellow_detected:
                self.get_logger().info('Tunnel sign detected. Moving ahead slowly...')
                self.kill_process()
                self.move_slow_forward()
                self.state = RaceState.ENTERING_TUNNEL

        elif self.state == RaceState.ENTERING_TUNNEL:
            pass  # No action needed; wait for scan_callback

    # Stop robot movement
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    # Move robot slowly forward
    def move_slow_forward(self):
        move_msg = Twist()
        move_msg.linear.x = 0.05
        self.cmd_vel_pub.publish(move_msg)

    # Callback when obstacle is detected
    def obstacle_callback(self, msg):
        if msg.data and not self.obstacle_flag:
            self.obstacle_flag = True

    # Callback when avoid obstacle goal is accepted
    def avoid_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('AvoidObstacle goal rejected.')
            return
        self.get_logger().info('AvoidObstacle goal accepted. Waiting for result...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.avoid_result_callback)

    # Callback after avoid obstacle result is received
    def avoid_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('AvoidObstacle succeeded.')
        else:
            self.get_logger().info('AvoidObstacle failed.')
        self.waiting_for_obstacle_done = False
        self.kill_process()
        self.launch_follow_road()
        self.state = RaceState.FOLLOW_ROAD_AGAIN

    # Image callback to detect yellow tunnel sign in second quadrant
    def image_callback(self, msg):
        if self.state != RaceState.FOLLOW_ROAD_AGAIN:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape
        second_quadrant = cv_image[0:height // 2, 0:width // 2]

        hsv = cv2.cvtColor(second_quadrant, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_pixels = cv2.countNonZero(mask)

        debug_img = cv2.bitwise_and(second_quadrant, second_quadrant, mask=mask)
        cv2.imshow('Second Quadrant Yellow Mask', debug_img)
        cv2.waitKey(1)

        self.get_logger().info(f'[DEBUG] Yellow pixel count in Q2: {yellow_pixels}')
        if yellow_pixels > 105 and not self.yellow_detected:
            self.yellow_detected = True

    # LIDAR scan callback to detect tunnel walls
    def scan_callback(self, msg):
        if self.state != RaceState.ENTERING_TUNNEL:
            return
        ranges = self.preprocess_scan(msg.ranges)
        left_range = np.nanmin(ranges[270:280])
        self.get_logger().info(f'[DEBUG] LIDAR left: {left_range:.3f}')
        if left_range < 0.2:
            self.get_logger().info('Tunnel walls detected close on left. Stopping robot.')
            self.stop_robot()
            self.kill_process()
            self.launch_navigation2()
            self.publish_initial_and_goal_pose()
            self.get_logger().info('Waiting for Navigation2 to stabilize...')
            time.sleep(5)
            self.get_logger().info('AutoraceReal node finished. You can now close it manually if needed.')

    # Replace NaN or zero values with infinity
    def preprocess_scan(self, ranges):
        return [r if r > 0.0 else float('inf') for r in ranges]

    # Launch the lane following node
    def launch_follow_road(self):
        self.kill_process()
        self.process = subprocess.Popen(['ros2', 'run', 'autorace_real', 'follow_road_real'], preexec_fn=os.setsid)

    # Launch the avoid obstacle node
    def launch_avoid_obstacle(self):
        self.kill_process()
        self.process = subprocess.Popen(['ros2', 'run', 'autorace_real', 'avoid_obstacle_real_action'], preexec_fn=os.setsid)

    # Launch Navigation2 system
    def launch_navigation2(self):
        self.get_logger().info('Launching navigation2...')
        if self.nav2_process is None:
            self.nav2_process = subprocess.Popen([
                'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                'map:=/home/sourav/iki_workspace/ros2_ws/src/25ssx2_sh-235839/stage_5/maps/entertunnelreal.yaml',
                'use_sim_time:=True'
            ], preexec_fn=os.setsid)
            self.get_logger().info(f'Nav2 launched with PID {self.nav2_process.pid}')
            time.sleep(5)

    # Publish initial and goal poses for navigation
    def publish_initial_and_goal_pose(self):
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
        goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', qos)

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        init_pose_pub.publish(initial_pose)
        self.get_logger().info('Published initial pose at (0,0) facing +X.')

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.6719
        goal_pose.pose.position.y = 1.4812
        goal_pose.pose.orientation.z = 0.6761
        goal_pose.pose.orientation.w = 0.7368
        goal_pose_pub.publish(goal_pose)
        self.get_logger().info('Published goal pose to (1.67, 1.41)')

        time.sleep(2)

    # Kill current running subprocess
    def kill_process(self):
        if self.process:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                time.sleep(0.5)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process = None

    # Clean shutdown procedure
    def shutdown(self):
        self.get_logger().info('Shutting down...')
        self.stop_robot()
        self.kill_process()

        if self.nav2_process:
            try:
                os.killpg(os.getpgid(self.nav2_process.pid), signal.SIGTERM)
                time.sleep(0.5)
                self.nav2_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Force killing nav2...')
                os.killpg(os.getpgid(self.nav2_process.pid), signal.SIGKILL)
            except Exception as e:
                self.get_logger().error(f'Nav2 process kill error: {e}')
            self.nav2_process = None

        cv2.destroyAllWindows()
        self.destroy_timer(self.timer)
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Destructor: force kill subprocesses if any remain
    def __del__(self):
        try:
            if self.nav2_process:
                os.killpg(os.getpgid(self.nav2_process.pid), signal.SIGKILL)
        except Exception:
            pass
        try:
            if self.process:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
        except Exception:
            pass

# Entry point of the program
def main(args=None):
    rclpy.init(args=args)
    node = AutoraceReal()

    # Register shutdown handler for SIGINT and SIGTERM
    def shutdown_handler(signum, frame):
        node.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unhandled Exception: {e}')
    finally:
        node.shutdown()

# Run main function
if __name__ == '__main__':
    main()

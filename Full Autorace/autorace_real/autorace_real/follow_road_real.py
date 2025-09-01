# Import ROS2 Python client library
import rclpy
from rclpy.node import Node

# ROS2 message types
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

# CVBridge for ROS <-> OpenCV conversion
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool


# QoS configuration for subscriptions
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class FollowLineNode(Node):
    def __init__(self):
        super().__init__('follow_line_node')

        # Set which color to follow ('yellow' or 'white')
        self.follow_color = 'white'

        # QoS setup
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.bridge = CvBridge()

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_publisher = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.has_published_obstacle = False
        self.obstacle_frames = 0
        # Add this flag to control obstacle publishing
        self.obstacle_detection_enabled = False

        # Start a timer to enable obstacle detection after 70 seconds
        self.create_timer(60.0, self.enable_obstacle_detection)



        # If following yellow line:
        if self.follow_color == 'yellow':
            # Subscriptions for camera and lidar
            self.subscription_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback_yellow, qos_profile)
            self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback_yellow, qos_profile)
            # Timer for control loop
            self.timer = self.create_timer(0.1, self.control_loop_yellow)

            # Parameters for obstacle detection and state
            self.safe_distance = 0.4
            self.obstacle_distance = 0.2
            self.latest_image = None
            self.yellow_golden_x = None
            self.state = 'BLOCKING'
            self.front_distance = float('inf')

        # If following white line:
        elif self.follow_color == 'white':
            # Subscriptions for camera and lidar
            self.subscription_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback_white, qos_profile)
            self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.scan_callback_white, qos_profile)
            # Timer for control loop
            self.timer = self.create_timer(0.1, self.process_white_line)

            # Control parameters
            self.kp = 0.015  # Proportional gain for steering
            self.forward_speed_max = 0.13
            self.forward_speed_min = 0.07
            self.max_error_for_speed = 15

            # State tracking
            self.nominal_offset_white = None
            self.nominal_offset_yellow = None
            self.latest_white_x = None
            self.latest_yellow_x = None
            self.image_center = None

            # Frame loss handling
            self.missing_white_frames = 0
            self.missing_yellow_frames = 0
            self.white_lost_threshold = 6
            self.yellow_lost_threshold = 6
            self.acceptable_missing_threshold = 2

            self.obstacle = 0.2
            self.prev_error = 0.0
            self.alpha = 0.3  # Smoothing factor for error
            self.front_distance = float('inf')
            self.waiting_for_clear_path = True

    ############################# yellow line process #############################################
    # Yellow line processing functions
    def image_callback_yellow(self, msg: Image):
        # Convert ROS image to OpenCV image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def lidar_callback_yellow(self, msg: LaserScan):
        # Clean invalid lidar values
        cleaned = [r if r > 0.0 else float('inf') for r in msg.ranges]
        # Check front distances
        front_vals = cleaned[0:20] + cleaned[340:360]
        self.front_distance = min(front_vals)

    def control_loop_yellow(self):
        # Skip if no image available
        if self.latest_image is None:
            return

        if self.front_distance == float('inf'):
            self.get_logger().info("[YELLOW] Waiting for first scan data...")
            return

        # Stop if obstacle very close
        if self.front_distance < self.obstacle_distance:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("[YELLOW] Obstacle ahead, stopping...")
            self.publisher.publish(twist)
            return

        # Process image ROI
        height, width, _ = self.latest_image.shape
        center_x = width // 2
        band_height = 5
        row_y_start = height - band_height - 5
        row_y_end = height - 5
        roi = self.latest_image[row_y_start:row_y_end, :, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Yellow mask using HSV
        yellow_mask = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))
        yellow_x = np.where(yellow_mask > 0)[1]

        # Display for debugging
        cv2.imshow('roi', roi)
        cv2.waitKey(1)

        twist = Twist()

        # State machine:
        if self.state == 'BLOCKING':
            if self.front_distance < self.safe_distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("[YELLOW] BLOCKING: Obstacle detected, stopping.")
            else:
                self.state = 'INIT'
                self.get_logger().info("[YELLOW] BLOCKING: Path clear, switching to INIT.")
            self.publisher.publish(twist)
            return

        if self.state == 'INIT':
            if len(yellow_x) > 0:
                # Set golden reference x-position
                self.yellow_golden_x = int(np.mean(yellow_x)) - 5
                self.state = 'TRACK_YELLOW'
                self.get_logger().info(f"[YELLOW] INIT: Yellow golden x set to {self.yellow_golden_x}")
            else:
                twist.linear.x = 0.0
                twist.angular.z = -0.1  # Rotate to find yellow
                self.get_logger().info("[YELLOW] INIT: Searching for yellow line.")
            self.publisher.publish(twist)
            return

        if self.state == 'TRACK_YELLOW':
            if len(yellow_x) > 0:
                yellow_mean = int(np.mean(yellow_x))
                delta = yellow_mean - self.yellow_golden_x
                target_x = center_x - delta
                error = (center_x - target_x)
                twist.linear.x = 0.07
                twist.angular.z = abs(float(error) / 100.0)
                self.get_logger().info(f"[YELLOW] TRACK_YELLOW: error={error}")
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.15  # Rotate to reacquire yellow
                self.get_logger().info("[YELLOW] TRACK_YELLOW: Lost yellow line, rotating to reacquire.")
            self.publisher.publish(twist)

    ################################ white line process ###########################################
    # White line processing functions
    def preprocess_scan(self, ranges):
        # Replace invalid LiDAR values with inf
        return [r if r > 0.0 else float('inf') for r in ranges]
    
    def enable_obstacle_detection(self):
        self.get_logger().info("[INFO] Obstacle detection enabled after 70 seconds.")
        self.obstacle_detection_enabled = True


    def scan_callback_white(self, msg: LaserScan):
        cleaned = self.preprocess_scan(msg.ranges)
        front_vals = cleaned[0:20] + cleaned[340:360]
        self.front_distance = min(front_vals)
        
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)
        self.get_logger().info('Robot stopped on shutdown.')


    def image_callback_white(self, msg: Image):
        # Process image to detect white and yellow edges
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        self.image_center = w // 2
        roi = frame[int(0.85 * h):, :]
        roi_h, roi_w, _ = roi.shape
        mid_x = roi_w // 2

        left_roi = roi[:, :mid_x]
        right_roi = roi[:, mid_x:]

        hsv_left = cv2.cvtColor(left_roi, cv2.COLOR_BGR2HSV)
        hsv_right = cv2.cvtColor(right_roi, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv_left, np.array([20, 100, 100]), np.array([30, 255, 255]))
        white_mask = cv2.inRange(hsv_right, np.array([0, 0, 200]), np.array([180, 40, 255]))

        # Compute centroids
        M_yellow = cv2.moments(yellow_mask)
        M_white = cv2.moments(white_mask)

        yellow_cx = int(M_yellow['m10'] / M_yellow['m00']) if M_yellow['m00'] > 0 else None
        white_cx = int(M_white['m10'] / M_white['m00']) + mid_x if M_white['m00'] > 0 else None

        # Update latest detections
        self.latest_yellow_x = yellow_cx if yellow_cx is not None else self.latest_yellow_x
        self.missing_yellow_frames = 0 if yellow_cx is not None else self.missing_yellow_frames + 1

        self.latest_white_x = white_cx if white_cx is not None else self.latest_white_x
        self.missing_white_frames = 0 if white_cx is not None else self.missing_white_frames + 1

        # Display overlay for debugging
        display = roi.copy()
        y_mid = display.shape[0] // 2
        if yellow_cx is not None:
            cv2.circle(display, (yellow_cx, y_mid), 7, (0, 255, 255), -1)
        if white_cx is not None:
            cv2.circle(display, (white_cx, y_mid), 7, (255, 255, 255), -1)
        if yellow_cx is not None and white_cx is not None:
            lane_cx = (yellow_cx + white_cx) // 2
            cv2.line(display, (yellow_cx, y_mid), (white_cx, y_mid), (0, 255, 0), 2)
            cv2.circle(display, (lane_cx, y_mid), 5, (0, 0, 255), -1)
        cv2.imshow("Lane ROI (left-yellow | right-white)", display)
        cv2.waitKey(1)

    def process_white_line(self):
        cmd = Twist()

        # Wait for clear path before moving
        if self.waiting_for_clear_path:
            if self.front_distance == float('inf'):
                self.get_logger().info("Waiting for first scan data...")
                return
            if self.front_distance < self.obstacle:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.publisher.publish(cmd)
                self.get_logger().info("Waiting for obstacle to be removed.")
                return
            else:
                self.waiting_for_clear_path = False
                self.get_logger().info("Path clear, starting navigation.")

        # Determine visible edges
        white_visible = (self.latest_white_x is not None and self.missing_white_frames <= self.acceptable_missing_threshold)
        yellow_visible = (self.latest_yellow_x is not None and self.missing_yellow_frames <= self.acceptable_missing_threshold)

        # Error calculation logic based on visible lines
        if white_visible and yellow_visible:
            lane_center_x = (self.latest_white_x + self.latest_yellow_x) / 2.0
            error_raw = lane_center_x - self.image_center
            context = "LANE"
            self.nominal_offset_white = self.latest_white_x - self.image_center
            self.nominal_offset_yellow = self.latest_yellow_x - self.image_center
        elif white_visible:
            if self.nominal_offset_white is None:
                self.nominal_offset_white = self.latest_white_x - self.image_center
                self.get_logger().info(f"[WHITE] Set nominal offset: {self.nominal_offset_white:+.1f}px")
            error_raw = (self.latest_white_x - self.image_center) - self.nominal_offset_white
            context = "WHITE-ONLY"
        elif yellow_visible:
            if self.nominal_offset_yellow is None:
                self.nominal_offset_yellow = self.latest_yellow_x - self.image_center
                self.get_logger().info(f"[YELLOW] Set nominal offset: {self.nominal_offset_yellow:+.1f}px")
            error_raw = (self.latest_yellow_x - self.image_center) - self.nominal_offset_yellow
            context = "YELLOW-ONLY"
        else:
            # Rotate or creep forward if both lines are lost
            if self.missing_white_frames > self.white_lost_threshold and self.missing_yellow_frames > self.yellow_lost_threshold:
                cmd.linear.x = 0.10
                cmd.angular.z = -0.2
                self.publisher.publish(cmd)
                self.get_logger().info("[LANE] Both edges lost - rotating to search.")
            else:
                cmd.linear.x = self.forward_speed_min
                cmd.angular.z = -0.10
                self.publisher.publish(cmd)
                self.get_logger().info("[LANE] Edges missing - creeping forward.")
            return

        # Apply filtered error for stability
        error = self.alpha * error_raw + (1 - self.alpha) * self.prev_error
        self.prev_error = error

        # Speed and steering control
        cmd.linear.x = self.adjust_speed_based_on_error(error)
        cmd.angular.z = -error * self.kp
        self.publisher.publish(cmd)
        self.get_logger().info(f"[{context}] error={error:+.2f}px speed={cmd.linear.x:.2f} m/s")
        
        # Obstacle detection publishing
        if self.obstacle_detection_enabled:
            if self.front_distance < 0.4:
                self.obstacle_frames += 1
            else:
                self.obstacle_frames = 0

            if self.obstacle_frames >= 3 and not self.has_published_obstacle:
                obstacle_msg = Bool()
                obstacle_msg.data = True
                self.obstacle_publisher.publish(obstacle_msg)
                self.get_logger().info("[INFO] Obstacle detected during lane follow (published).")
                self.has_published_obstacle = True
            elif self.obstacle_frames == 0:
                self.has_published_obstacle = False


    def adjust_speed_based_on_error(self, error):
        # Adjust speed based on steering error
        scale = max(0.0, min(1.0, 1 - abs(error) / self.max_error_for_speed))
        return self.forward_speed_min + scale * (self.forward_speed_max - self.forward_speed_min)

# Entry point

def main(args=None):
    rclpy.init(args=args)
    node = FollowLineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

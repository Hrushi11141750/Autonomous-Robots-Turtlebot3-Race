#!/usr/bin/env python3

# Import ROS2 core modules
import rclpy  
from rclpy.node import Node  
from rclpy.callback_groups import ReentrantCallbackGroup  
from rclpy.executors import MultiThreadedExecutor  
from rclpy.action import ActionClient  

# Import message and service types
from sensor_msgs.msg import Image  
from geometry_msgs.msg import PoseStamped  
from nav2_msgs.action import NavigateToPose  
from autorace_real_interfaces.srv import DetectTunnelSign  

# Import OpenCV bridge and image processing libraries
from cv_bridge import CvBridge  
import cv2  
import numpy as np  
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  

class EnterTunnelNode(Node):
    def __init__(self):
        super().__init__('enter_tunnel_real')  # Initialize the ROS2 node with the name

        self.bridge = CvBridge()  # Used to convert ROS image messages to OpenCV images

        self.callback_group = ReentrantCallbackGroup()  # Allows concurrent callbacks (image, service, action)

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)  # QoS profile for camera

        self.sign_detected = False  # Flag indicating if the tunnel sign has been detected
        self.detection_active = False  # Flag indicating if detection is currently active

        # Subscribe to camera image topic
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile,
            callback_group=self.callback_group
        )

        # Create Nav2 action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group
        )

        # Create a service server to trigger detection activation
        self.service = self.create_service(
            DetectTunnelSign, 'detect_tunnel_sign',
            self.handle_detect_tunnel_service,
            callback_group=self.callback_group
        )

        self.get_logger().info(" EnterTunnelNode initialized. Waiting for service call to begin detection...")

    def handle_detect_tunnel_service(self, request, response):
        """Service callback to start detection when the service is called."""
        self.detection_active = True  # Enable detection mode
        response.success = True  # Indicate successful handling of the service call
        response.message = "Tunnel detection started. Monitoring image stream..."
        self.get_logger().info(" Tunnel detection activated via service.")
        return response

    def image_callback(self, msg):
        """Callback to process incoming camera frames and detect the tunnel sign."""
        if not self.detection_active or self.sign_detected:
            return  # Do nothing if detection not active or already detected

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Convert ROS2 image to OpenCV image
        height, width, _ = frame.shape  # Get image dimensions
        half_h, half_w = height // 2, width // 2  # Compute half dimensions

        q1_roi = frame[0:half_h, 0:half_w]  # Extract top-left quadrant (Q1)
        zoomed_roi = cv2.resize(q1_roi, (width, height), interpolation=cv2.INTER_LINEAR)  # Zoom Q1 for consistency

        hsv = cv2.cvtColor(q1_roi, cv2.COLOR_BGR2HSV)  # Convert Q1 to HSV color space

        # Define yellow detection range for HSV
        lower_yellow = np.array([20, 120, 120])
        upper_yellow = np.array([35, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  # Create mask for yellow color
        yellow_area = cv2.countNonZero(mask)  # Count yellow pixels detected

        self.get_logger().info(f" Yellow area detected = {yellow_area}")  # Log the yellow pixel count

        if 70 < yellow_area < 1000:  # Threshold for valid tunnel sign detection
            self.sign_detected = True
            self.get_logger().info(' Tunnel sign detected! Sending Nav2 goal...')
            self.send_nav2_goal()  # Send the navigation goal
        else:
            self.get_logger().info(" Tunnel sign not detected in Q1")  # Log if not detected

        # Debug visualization for monitoring detection
        try:
            zoomed = cv2.resize(q1_roi, (width, height))
            cv2.imshow("Q1 Zoomed", zoomed)
            cv2.imshow("Yellow Mask", mask)
            cv2.waitKey(1)
        except:
            pass

    def send_nav2_goal(self):
        """Send a navigation goal forward when the tunnel sign is detected."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(" Nav2 action server not available!")
            return  # Exit if Nav2 server is not ready

        goal_msg = NavigateToPose.Goal()  # Create Nav2 goal message
        goal_msg.pose.header.frame_id = 'map'  # Set the frame of reference
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp

        goal_msg.pose.pose.position.x = 0.9  # Move ~0.8 meters forward
        goal_msg.pose.pose.position.y = 0.05  # Slight y offset
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward (no rotation)

        self.get_logger().info(" Sending Nav2 goal to move forward...")  # Log goal sending
        self.nav_to_pose_client.send_goal_async(goal_msg)  # Send the goal asynchronously


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 Python environment
    node = EnterTunnelNode()  # Instantiate the node

    executor = MultiThreadedExecutor()  # Use a multithreaded executor
    executor.add_node(node)  # Add the node to the executor

    try:
        executor.spin()  # Keep the node spinning
    except KeyboardInterrupt:
        node.get_logger().info(" Shutting down...")  # Handle Ctrl+C gracefully
    finally:
        node.destroy_node()  # Clean up node
        rclpy.shutdown()  # Shut down ROS2


if __name__ == '__main__':
    main()





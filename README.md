# TurtleBot3 Autonomous Navigation – ROS2 Project

This project demonstrates **real-time autonomous navigation** for the TurtleBot3 robot using **ROS2 Humble**. The system integrates **lane following**, **obstacle avoidance**, and **tunnel navigation** using sensor fusion from **camera and LiDAR** inputs. It leverages the **Nav2 stack** for path planning and SLAM for partially structured environments.

---

## Project Overview

The autonomous navigation pipeline is implemented as three major modules:

### 1. Lane Following (Follow Road)
- **Purpose:** Allows the TurtleBot3 to follow lanes marked with **yellow** or **white lines** in real time.
- **Input:** Camera images from `/camera/image_raw` and LiDAR scans from `/scan`.
- **Output:** Velocity commands (`/cmd_vel`) to maintain lane alignment.
- **Implementation:**
  - Converts camera images to HSV color space for detecting yellow and white lines.
  - Computes lane center using pixel centroid analysis.
  - Controls robot steering using proportional control based on lane offset.
  - Uses LiDAR data to detect obstacles and stops or adjusts speed accordingly.
  - Implements lost-line recovery by rotating or creeping forward when lines are not detected.
  - Includes separate handling for yellow and white line modes to support race tracks, tunnels, or open areas.

### 2. Obstacle Avoidance
- **Purpose:** Ensures safe navigation by detecting obstacles and adjusting the robot's motion.
- **Input:** LiDAR scans and optional camera input for line detection.
- **Output:** Adjusted velocity commands to avoid collisions.
- **Implementation:**
  - Uses LiDAR to calculate distances ahead and to the right of the robot.
  - Performs wall-following when obstacles are detected close by.
  - Integrates an **Action Server** to asynchronously handle obstacle avoidance maneuvers.
  - Supports turning sequences to bypass obstacles while preserving lane alignment.
  - Continuously monitors environment and dynamically resumes normal lane following after clearing obstacles.

### 3. Tunnel Navigation (Enter Tunnel)
- **Purpose:** Detects tunnel signs and safely guides the TurtleBot3 into tunnels.
- **Input:** Camera feed for tunnel sign detection.
- **Output:** Nav2 goals to move forward inside the tunnel and velocity commands for controlled movement.
- **Implementation:**
  - Subscribes to camera images and detects **yellow signs** in specific quadrants.
  - Uses HSV thresholding to identify yellow areas as tunnel markers.
  - Sends a **Nav2 goal** to move forward when the tunnel sign is detected.
  - Works with **Reentrant Callback Groups** to allow concurrent image processing, action execution, and service handling.
  - Can be triggered dynamically using a ROS2 service for tunnel detection.

---

## Key Features

1. **Real-Time Autonomy:** The robot responds dynamically to obstacles, lane curvature, and tunnel entries without manual intervention.  
2. **Multithreading & Callback Groups:** Ensures concurrent processing of sensor data and actions, improving performance and reducing latency.  
3. **SLAM Integration:** Supports partially structured environments by leveraging map-based localization and Nav2 planning.  
4. **Portable Deployment:** Nodes can be launched using ROS2 launch files or containerized with Docker for consistent testing.  
5. **Adaptive Control:** Lane following adapts linear speed and steering based on detected lane offset and obstacle proximity.  
6. **Robust Obstacle Recovery:** Action servers and state machines handle obstacle encounters gracefully, including 90-degree turns and wall-following strategies.  
7. **Tunnel Awareness:** Combines visual detection and navigation goals to ensure safe tunnel traversal, even in partially constrained spaces.

---

## Implementation Notes

- **Sensor Fusion:** LiDAR data is preprocessed to replace invalid measurements and integrated with camera data to detect obstacles and lane boundaries.  
- **State Machine Logic:** The main node uses a **state machine** to switch between `FOLLOW_ROAD`, `AVOID_OBSTACLE`, and `ENTER_TUNNEL` states.  
- **Image Processing:** OpenCV and CVBridge are used for real-time image analysis, including color segmentation, region-of-interest selection, and pixel centroid calculation.  
- **Action Servers:** Obstacle avoidance and tunnel entry leverage ROS2 action servers for asynchronous execution and goal monitoring.  
- **Velocity Control:** Linear and angular velocities are published to `/cmd_vel`, with proportional steering corrections based on detected errors.  
- **Visualization:** Debug windows display lane detection and masked images for development and tuning.

---

## How the Project Works

1. The robot starts in **lane-following mode**, detecting yellow or white lines and aligning itself using a proportional controller.  
2. **LiDAR continuously monitors obstacles**; if an obstacle is detected within a safe range, the obstacle avoidance action server is triggered.  
3. **Obstacle avoidance maneuvers** include wall-following, turning sequences, and resuming lane alignment.  
4. **Tunnel detection** uses a dedicated node to detect yellow tunnel signs; upon detection, a Nav2 goal is sent to move the robot safely into the tunnel.  
5. **After clearing the tunnel**, the robot automatically resumes lane following and obstacle-aware navigation.

---

## Benefits & Use Cases

- Ideal for **autonomous race tracks** or structured indoor environments.  
- Provides a **complete framework** for integrating lane following, obstacle avoidance, and goal-directed navigation.  
- Can be adapted for **warehouse robots**, **delivery robots**, or **autonomous mobile platforms**.  
- Demonstrates **real-world integration of ROS2, computer vision, SLAM, and multithreaded robotics control**.

---

This project serves as a **reference implementation** for building a fully autonomous mobile robot using ROS2. By understanding the lane following, obstacle avoidance, and tunnel entry mechanisms, a developer can recreate or extend this system for other robot platforms or environments.



https://github.com/user-attachments/assets/71fce91d-068d-4664-b585-f0ce15dad198







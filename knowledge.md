# Repository Knowledge Base

This document is a deep map of what is in this repository, where it lives, how pieces connect, and what is currently complete vs. scaffolded.

## 1. High-level repository purpose

This is a ROS 2 workspace-style repository for spatial transforms and robot navigation exercises. It contains four C++ ROS 2 packages:

- spatial_utils: shared transform conversion utilities.
- spatial_transforms: interactive transform broadcaster and transform listener examples.
- nav_goals: Nav2 action client example that sends a goal.
- follower_robot: follower behavior scaffold that reacts to AprilTag detections and issues navigation goals.

The code appears educational/homework-oriented in several places (explicit TODO text and instructional comments).

## 2. Top-level structure

- README.md
  - Environment/bootstrap notes for ROS 2 Humble workspace setup and dependencies.
  - Includes package clone/build commands for a larger robotics stack.
- follower_robot/
  - ROS 2 package with node + helper class for AprilTag-following behavior.
- nav_goals/
  - ROS 2 package with one executable that sends a NavigateToPose goal.
- spatial_transforms/
  - ROS 2 package with TF broadcaster + listener examples.
- spatial_utils/
  - ROS 2 package that exports transform utility functions.
- .vscode/settings.json
  - File association hints for C++ headers.

## 3. Package-by-package breakdown

## 3.1 spatial_utils package

Location: spatial_utils/

### Intent

Provide reusable conversion helpers between ROS TransformStamped messages and Eigen 4x4 rigid transforms.

### Build metadata

- CMake: spatial_utils/CMakeLists.txt
  - Builds library named spatial_utils from src/transform_util.cpp.
  - Exports include directory and library via ament.
  - Dependencies: rclcpp, tf2_ros, geometry_msgs, Eigen3.
- Manifest: spatial_utils/package.xml
  - Declares build tool ament_cmake.
  - Declares same core dependencies.
  - Description/license still TODO.

### Source files

- include/spatial_utils/transform_util.h
  - Declares:
    - transformToMatrix
    - matrixToTransform
    - printTransform
- src/transform_util.cpp
  - printTransform implemented.
  - transformToMatrix currently placeholder (returns identity).
  - matrixToTransform currently placeholder (returns mostly uninitialized TransformStamped).

### Operational status

- Utility print function works.
- Core conversion functions are incomplete; any package relying on real transform math from these helpers is currently functionally blocked.

## 3.2 spatial_transforms package

Location: spatial_transforms/

### Intent

Demonstrate TF broadcasting and lookup, plus composing offset frames using Eigen.

### Build metadata

- CMake: spatial_transforms/CMakeLists.txt
  - Builds two executables:
    - spatial_transform_node
    - tf_listener_node
  - Links GTK3 into spatial_transform_node for GUI sliders.
  - Depends on spatial_utils, rclcpp, tf2_ros, geometry_msgs, Eigen3.
- Manifest: spatial_transforms/package.xml
  - Declares package as ROS2 transforms package.
  - Depends on spatial_utils, rclcpp, tf2/tf2_ros/tf2_eigen, eigen.

### Docs and examples

- spatial_transforms/README.md
  - Minimal title-only README.
- spatial_transforms/examples.txt
  - Workflow notes for running broadcaster/listener and inspecting TF tree.

### Source files

- src/spatial_transform_node.cpp
  - Node: SpatialTransformNode
    - Broadcasts transform world -> example_frame at 20 Hz via timer.
    - Uses translation variables x_, y_, z_ and Eigen quaternion q_.
  - GUI: TransformGUI
    - GTK sliders for x, y, z, rx, ry, rz.
    - Updates node transform values in real time.
  - main
    - Runs ROS spin_some and GTK event iteration in a loop.

- src/tf_listener_node.cpp
  - Node: TFListenerNode
    - Maintains tf_buffer_, tf_listener_, tf_broadcaster_.
    - Looks up world -> example_frame transform.
    - Prints transform with printTransform from spatial_utils.
    - Builds two additional offsets using Eigen 4x4 matrices:
      - off1 relative to example_frame (translation + rotation around X).
      - off2 composed after off1.
    - Broadcasts resulting transforms each loop.

### Operational status

- Broadcaster/listener architecture is complete.
- Correctness of matrix conversion path depends on spatial_utils TODO implementations.

## 3.3 nav_goals package

Location: nav_goals/

### Intent

Simple action client for Nav2 NavigateToPose with TF conversion from base_link-relative goal to map frame.

### Build metadata

- CMake: nav_goals/CMakeLists.txt
  - Builds executable send_goal.
  - Depends on rclcpp, rclcpp_action, nav2_msgs, geometry_msgs, std_msgs, tf2, tf2_ros, tf2_geometry_msgs.
- Manifest: nav_goals/package.xml
  - Dependency declarations match implementation intent.
  - Description/license still TODO.

### Docs and examples

- nav_goals/examples.txt
  - Single launch command for TurtleBot4 ignition bringup with SLAM/Nav2/RViz.

### Source file

- src/send_goal.cpp
  - Node: NavGoalSender
    - Sets use_sim_time true.
    - Creates NavigateToPose action client and waits for server.
    - Constructs a local goal pose in base_link (x=1.0, y=0.0, orientation.w=1.0).
    - Looks up map <- base_link transform.
    - Uses tf2::doTransform to convert local goal into map frame.
    - Sends async nav goal.
  - main loop retries send_goal until transform lookup succeeds once.

### Operational status

- Mostly complete example client.
- The current loop exits when a goal is successfully sent, not when navigation completes.

## 3.4 follower_robot package

Location: follower_robot/

### Intent

Follower behavior that tracks AprilTag ID 1, computes a target pose at follow distance, and sends navigation goals.

### Build metadata

- CMake: follower_robot/CMakeLists.txt
  - Builds executable follower_robot from:
    - src/FollowerRobotNode.cpp
    - src/MoveToTarget.cpp
    - src/follower_robot.cpp
  - Dependencies include apriltag_msgs, cv_bridge, geometry_msgs, nav2_msgs, rclcpp, rclcpp_action, sensor_msgs, spatial_utils, tf2_ros, OpenCV.
- Manifest: follower_robot/package.xml
  - Declares core deps but has a likely typo:
    - spatial_util (singular) is listed, while package name in repo is spatial_utils.
  - Description/license still TODO.

### Header files

- include/follower_robot/FollowerRobotNode.h
  - Class FollowerRobotNode inherits rclcpp::Node.
  - Declares:
    - AprilTag callback.
    - motion/change detection helper (theTagMoved).
    - computeAndAct control flow.
    - geometry helpers to implement.
  - Holds TF listener/buffer/broadcaster and MoveToTarget helper.

- include/follower_robot/MoveToTarget.h
  - Class MoveToTarget wraps Nav2 goal sending.
  - Constructor takes node pointer.
  - Declares goal response and result callbacks.
  - Declares copyToGoalPoseAndSend (to implement).

### Source files

- src/FollowerRobotNode.cpp
  - Constructor creates subscriber on /apriltag_detections.
  - aprilTagCallback filters detections for tag id == 1.
  - theTagMoved is implemented:
    - Composes map->base_link and base_link->tag transforms.
    - Compares current tag position against previous tag position in map frame.
    - Uses tag_motion_threshold_ to decide movement.
  - computeGoToFrameFromBaseLink is placeholder (returns identity currently).
  - computeDistanceBaseLinkTag1 is placeholder (returns 0.0 currently).
  - computeAndAct scaffold exists with key TODOs:
    - lookupTransform calls not yet implemented.
    - m_map_to_go_to_ composition not filled.
    - broadcast transform message assembly not filled.

- src/MoveToTarget.cpp
  - Constructor and callback binding implemented.
  - Waits for navigate_to_pose action server.
  - copyToGoalPoseAndSend currently placeholder:
    - logs "Sending goal!"
    - creates goal message but does not fill pose fields.

- src/follower_robot.cpp
  - Standard ROS node entrypoint for FollowerRobotNode.

### Operational status

- This package is intentionally partial and appears to be a homework template.
- Many key runtime behaviors are not yet implemented.

## 4. Cross-package architecture and data flow

Intended pipeline appears to be:

1. AprilTag detections arrive in follower_robot from /apriltag_detections.
2. follower_robot uses TF lookups to compute tag position relative to robot and map.
3. follower_robot computes a go-to pose that keeps robot follow_distance from tag.
4. follower_robot sends Nav2 NavigateToPose goals via MoveToTarget.
5. spatial_utils provides transform <-> matrix conversion for geometry math.
6. spatial_transforms package acts as standalone learning/demo utilities for TF and Eigen.
7. nav_goals package acts as standalone learning/demo utility for sending goals.

Current reality:

- The helper math package spatial_utils is not complete yet.
- follower_robot core action math and command publishing are also not complete.
- nav_goals is the most complete end-to-end executable in this repo.

## 5. Build and environment notes

From top-level README.md:

- Targets ROS 2 Humble setup.
- Includes dependency setup for gtk and apriltag libraries.
- Includes colcon build workflow and rosdep install command.
- Uses workspace variable COLCON_WS and common robotics repositories.

Because this repository is one part of a larger robotics stack, runtime success likely depends on external packages and robot/simulator bringup availability.

## 6. Known inconsistencies and risks

1. Incomplete functions blocking behavior:
- spatial_utils/src/transform_util.cpp placeholders.
- follower_robot/src/FollowerRobotNode.cpp placeholders/TODOs.
- follower_robot/src/MoveToTarget.cpp placeholder send logic.

2. Dependency naming mismatch:
- follower_robot/package.xml uses spatial_util, but package is spatial_utils.

3. Metadata still scaffolded:
- TODO descriptions/licenses in multiple package.xml files.

4. Potential include/dependency drift:
- Some dependencies are listed in CMake but not fully mirrored in package.xml (and vice versa).

## 7. Quick file index with purpose

- README.md: Workspace setup and build instructions.
- .vscode/settings.json: Header association preferences.

follower_robot/
- CMakeLists.txt: Build executable and dependencies.
- package.xml: ROS package manifest (contains spatial_util typo).
- include/follower_robot/FollowerRobotNode.h: Main follower node class declaration.
- include/follower_robot/MoveToTarget.h: Nav goal helper class declaration.
- src/FollowerRobotNode.cpp: Main follower logic scaffold + tag movement detection.
- src/MoveToTarget.cpp: Action client wrapper scaffold.
- src/follower_robot.cpp: Node entrypoint.

nav_goals/
- CMakeLists.txt: Build send_goal executable.
- package.xml: ROS package metadata.
- examples.txt: Example launch command.
- src/send_goal.cpp: Nav2 goal sender with TF conversion.

spatial_transforms/
- CMakeLists.txt: Builds transform broadcaster/listener executables.
- package.xml: ROS package metadata.
- README.md: Minimal package description.
- examples.txt: Suggested exercise steps.
- src/spatial_transform_node.cpp: GTK-driven TF broadcaster.
- src/tf_listener_node.cpp: TF lookup + offset-frame broadcaster.

spatial_utils/
- CMakeLists.txt: Builds and exports utility library.
- package.xml: ROS package metadata.
- include/spatial_utils/transform_util.h: Utility function declarations.
- src/transform_util.cpp: Utility implementations (partially TODO).

## 8. Suggested next engineering steps

1. Complete transform utility conversions in spatial_utils first.
2. Implement follower_robot geometry computations and TF lookup/broadcast flow.
3. Complete MoveToTarget::copyToGoalPoseAndSend with full PoseStamped mapping.
4. Fix package.xml dependency typo (spatial_util -> spatial_utils).
5. Replace TODO metadata fields in package manifests.
6. Add run instructions per package (commands + expected topics/frames) into package READMEs.

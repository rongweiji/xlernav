# xlernav Roadmap

This monorepo integrates Raspberry Pi (onboard) and WSL (server/GPU) ROS 2 stacks
for lekiwi-based robots using Nav2, with optional SLAM, depth, and 3D mapping.
The Pi is treated as a thin client for sensors and actuators; WSL performs
compute-heavy mapping, planning, and visualization.

## Goals

- Provide a reliable ROS 2 bridge between Pi and WSL over Wi-Fi.
- Stream stereo images and robot telemetry to WSL in real time.
- Run SLAM, depth, and navigation in WSL with live visualization.
- Close the control loop: WSL sends velocity/goal commands to the Pi.
- Support optional 3D reconstruction with nvblox for richer maps.

## Key Technologies

- ROS 2 (DDS networking across Pi and WSL)
- Nav2 for planning, control, and behavior trees
- ORB-SLAM3 for visual SLAM
- Depth-Anything-3 for dense depth prediction
- nvblox (optional) for 3D mapping/TSDF/ESDF
- RViz for visualization and debugging

## System Architecture (High Level)

- **Raspberry Pi (onboard):**
  - Stereo camera drivers + calibration
  - Wheel odometry and motor control
  - Publishes sensor topics; subscribes to control commands
- **WSL (server/GPU):**
  - SLAM + depth + map fusion
  - Nav2 stack + planners + controllers
  - RViz + diagnostics

## Phased Plan

### Phase 0: Repository Baseline

- Decide ROS 2 distro (e.g., Humble or Iron) for both Pi and WSL.
- Define folder layout in this monorepo (pi/, wsl/, shared/, docs/).
- Document environment setup and requirements.

### Phase 1: ROS 2 Networking + Live Camera

- Install ROS 2 on Pi (Docker) and WSL with the same distro.
- Configure DDS to work over Wi-Fi (domain ID, discovery settings).
- Publish stereo camera topics from Pi; subscribe/visualize in WSL.
- Verify time sync (chrony/NTP) and topic latency.
- Deliverable: RViz on WSL shows live stereo images.

### Phase 2: Control Loop

- Implement/bridge wheel odometry on Pi (nav_msgs/Odometry).
- Define cmd_vel interface on Pi to drive motors.
- WSL publishes cmd_vel to Pi; Pi acknowledges with odom.
- Deliverable: teleop from WSL reliably drives the robot.

### Phase 3: SLAM Integration

- Run ORB-SLAM3 in WSL using stereo topics from Pi.
- Publish TF tree and map/pose topics for Nav2.
- Validate stability, loop closure, and map quality.
- Deliverable: WSL publishes map + base_link pose.

### Phase 4: Nav2 Bringup

- Configure Nav2 for robot footprint, sensors, and costmaps.
- Integrate map/odom/TF from SLAM and wheel odometry.
- Tune planners/controllers for the lekiwi platform.
- Deliverable: goal-based navigation in a test environment.

### Phase 5: Depth + 3D Mapping (Optional)

- Run Depth-Anything-3 in WSL for dense depth.
- Fuse depth + pose into nvblox for 3D ESDF/TSDF maps.
- Use 3D costmap layers if beneficial.
- Deliverable: 3D map visualization and optional 3D-aware planning.

### Phase 6: Reliability + Productization

- Add launch files, systemd services, and health checks.
- Add logging, diagnostics, and remote restart flow.
- Document troubleshooting and performance tuning.

## Decisions Needed

- ROS 2 distro selection and supported OS versions.
- Topic names, TF tree conventions, and namespace strategy.
- Whether Pi should do any local preprocessing (e.g., image rectification).
- Whether to use ros2_bridge or stay pure ROS 2.

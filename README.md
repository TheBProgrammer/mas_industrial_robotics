# Skratch Robot Workspace

This workspace contains the complete ROS 2 software stack for the Skratch robot, including simulation, localization, and navigation capabilities.

## Packages Overview

### 1. **skratch_simulation**
- **Purpose**: Provides Gazebo simulation and mapping resources.
- **Sub-packages**:
  - `skratch_gazebo`: Gazebo worlds and launch files.
  - `skratch_mapping`: SLAM configurations and saved maps.

### 2. **skratch_localization**
- **Purpose**: Handles robot localization using AMCL.
- **Features**:
  - Uses Lidar scan data to position the robot on a static map.
  - Publishes the `map -> odom` transform.

### 3. **skratch_navigation**
- **Purpose**: Provides the Nav2 navigation stack (Planning & Control).
- **Features**:
  - **MPPI Controller**: Predictive path tracking with `nav2_params_mppi_theta.yaml`.
  - **ThetaStar Planner**: Efficient global path planning.
  - **Pose Management**: Tools to save and navigate to waypoints.

### 4. **skratch_sensor**
- **Purpose**: Sensor data integration.
- **Features**:
  - **Dual Laser Merger**: Merges front and rear LiDAR scans into a single topic.

### 5. **skratch_description**
- **Purpose**: Robot URDF/Xacro model and meshes.

---

## Installation

Install dependencies from the workspace root (`~/skratch_ws`):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## How to Run the Full Stack

Follow these steps to get everything running. You can use **separate terminals** for each step.

### Step 1: Start Simulation
Launch the robot in Gazebo (includes dual laser merger).
```bash
launch_skratch_sim
# OR: ros2 launch skratch_gazebo gazebo.launch.py
```

### Step 2: Start Localization
Localize the robot on the map.
```bash
localise_skratch
# OR: ros2 launch skratch_localization localization.launch.py
```

### Step 3: Start Navigation
Enable the Nav2 stack.
```bash
enable_skratch_navigation
# OR: ros2 launch skratch_navigation nav2.launch.py
```

### Setup: Record Poses
Before navigating, you need to define some target locations.
1. Move the robot to a desired spot (using teleop or GUI).
2. Run the save tool:
```bash
save_poses
# OR: ros2 run skratch_navigation save_poses --ros-args -p use_sim_time:=true
```
3. Enter a name (e.g., `WS01`) and press Enter.

### Step 4: Navigate to Pose
Send the robot to one of your saved locations.
```bash
navigate WS01
# OR: ros2 run skratch_navigation navigate WS01 --ros-args -p use_sim_time:=true
```

---

## Command Reference

We have configured convenient aliases for common operations. You can use either method.

### 1. Simulation
Start the Gazebo world and spawn the robot.

| Method | Command |
|--------|---------|
| **Alias** | `launch_skratch_sim` |
| **Raw** | `ros2 launch skratch_gazebo gazebo.launch.py` |

### 2. Localization
Start AMCL to locate the robot in the map.

| Method | Command |
|--------|---------|
| **Alias** | `localise_skratch` |
| **Raw** | `ros2 launch skratch_localization localization.launch.py` |

### 3. Navigation
Start the Nav2 stack for autonomous movement.

| Method | Command |
|--------|---------|
| **Alias** | `enable_skratch_navigation` |
| **Raw** | `ros2 launch skratch_navigation nav2.launch.py` |

---

## Pose Management Tools

We have created custom tools to help manage navigation goals.

### **Save Poses**
Interactively save your current robot location to a file.

| Method | Command (Simulation) |
|--------|----------------------|
| **Alias** | `save_poses` |
| **Raw** | `ros2 run skratch_navigation save_poses --ros-args -p use_sim_time:=true` |

**How to use:**
1. Move the robot to a target location (e.g. using Teleop).
2. Run the command.
3. Enter a name for the pose (e.g., "Kitchen").
4. Repeat for other locations.

### **Navigate to Pose**
Send the robot to a saved pose by name.

| Method | Command (Simulation) |
|--------|----------------------|
| **Alias** | `navigate <POSE_NAME>` |
| **Raw** | `ros2 run skratch_navigation navigate <POSE_NAME> --ros-args -p use_sim_time:=true` |

*(Note: `move_base <POSE>` is also available as an alias for `navigate`)*

### **List Poses**
View all saved poses.

| Method | Command |
|--------|---------|
| **Alias** | `list_poses` |
| **Raw** | `cat ~/skratch_ws/src/skratch_navigation/config/navigation_goals.yaml` |

---

## Setup

To enable the aliases, run:
```bash
~/ros_robots/skratch_ws/src/setup_aliases.sh
source ~/.bashrc
```

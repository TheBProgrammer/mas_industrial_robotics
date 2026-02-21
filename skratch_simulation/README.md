# Skratch Simulation Package

This repository contains the simulation environment and mapping resources for the Skratch robot.
It is composed of two main sub-packages:
1. **`skratch_gazebo`**: Gazebo simulation worlds and launch files.
2. **`skratch_mapping`**: SLAM configurations and saved maps.

---

## Requirements
* Ubuntu 22.04 (Jammy Jellyfish)
* ROS 2 Humble ([see installation guide](https://docs.ros.org/en/humble/Installation.html))

## Dependencies
```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs gazebo libgazebo-dev python3-colcon-common-extensions ros-humble-xacro ros-humble-joint* \
ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard
```

---

## Setup Instructions

### 1. Create a ROS 2 Workspace
```bash
mkdir -p ~/ros_robots/skratch_ws/src
cd ~/ros_robots/skratch_ws/src
```

### 2. Clone Required Repositories

#### 2.1 Clone `skratch_description` (URDF)
```bash
git clone -b dev_classic --single-branch https://github.com/b-it-bots/skratch_description.git
```

#### 2.2 Clone the Simulation Package
```bash
git clone -b dev_classic --single-branch https://github.com/b-it-bots/skratch_simulation.git
```

### 3. Build the Workspace

Navigate to the workspace root and build:
```bash
cd ~/ros_robots/skratch_ws
colcon build --symlink-install --packages-select skratch_description skratch_gazebo skratch_mapping
source install/setup.bash
```

---

## Running Simulation

### 1. Launch Gazebo Simulation

Source the setup file and launch Gazebo:
```bash
ros2 launch skratch_gazebo gazebo.launch.py
```

This will launch:
- Gazebo environment (default: `atwork_world.sdf`)
- Robot model spawned at origin
- **Dual Laser Merger** (merges front/rear scans)
- RViz visualization

### 2. Control the Robot
Once the robot is launched, you can use teleop twist keyboard to control it:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Mapping (SLAM)

The `skratch_mapping` package handles map creation and storage.

### Creating a Map

1. Launch simulation (see above).
2. Run the SLAM toolbox (online async mode):
   ```bash
   ros2 launch skratch_mapping online_async.launch.py
   ```
3. Drive the robot around using teleop to cover the area.

### Saving the Map

Once the map is complete, save it to the maps directory:

```bash
cd ~/ros_robots/skratch_ws/src/skratch_simulation/skratch_mapping/maps/
ros2 run nav2_map_server map_saver_cli -f my_new_map
```

### Available Maps
The package comes with pre-built maps located in `skratch_mapping/maps/`:
- **`rc_arena_sim`**: The RoboCup @Work arena map.
- **`eval_arena`**: Evaluation arena map.

---

## Running in Docker

To run the simulation in a Docker container:

1. **Build Docker Image**
   Choose the dockerfile based on your hardware:
   ```bash
   docker build -t skratch_sim:latest -f dockerfiles/Dockerfile.rocm dockerfiles
   ```

2. **Run Container**
   * **Wayland** display with AMD GPU:
     ```bash
     ./sim-docker-amd-wayland.sh
     ```
   * **NVIDIA GPU**:
     (Configuration pending)
  

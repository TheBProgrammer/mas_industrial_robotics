# Skratch Description Package

## Overview

This package contains the complete description of the Skratch robot used in the RoboCup @Work League.
It defines the robot's physical structure, geometry, joints, links, and visual appearance using URDF/Xacro, along with the required mesh files.

The robot model is designed to be used with ROS 2, and is fully compatible with Gazebo for simulation and RViz for visualization.

## Package Contents

This package includes the following components:

```
skratch_description/
├── config/
│   └── skratch_description.rviz
├── description/
│   ├── base/
│   │   ├── kinova_arm_base_plate.xacro
│   │   └── skratch_base.xacro
│   ├── sensor_mounts/
│   │   └── lidar_mount.xacro
│   ├── sensors/
│   │   └── hokuyo_urg_04lx_ug01.xacro
│   └── wheels/
│       ├── common.xacro
│       ├── kelo_drive_mid.urdf.xacro
│       ├── kelo_drive_top.urdf.xacro
│       ├── kelo_drive_wheel.urdf.xacro
│       └── materials.urdf.xacro
├── gazebo/
│   ├── gazebo_ros_control.xacro
│   └── gazebo_ros_planar_move.xacro
├── launch/
│   └── description.launch.py
├── meshes/
│   ├── base/
│   │   ├── arm_base_plate.stl
│   │   └── skratch_base.stl
│   ├── kelo_drive/
│   │   ├── kelo_drive_mid_part.dae
│   │   ├── kelo_drive_top_part.dae
│   │   ├── kelo_drive_wheel.dae
│   │   └── kelo_drive_wheel.stl
│   ├── mounts/
│   │   └── lidar_mount.stl
│   └── sensors/
│       └── Hokuyo_URG_04LX_UG01.stl
├── urdf/
│   └── skratch_urdf.xacro
├── CMakeLists.txt
└── package.xml
```

## Robot Model Description

- **Robot type**: Mobile robot with custom Kelo wheels base
- **Description format**: URDF / Xacro
- **Links & joints**: Defined according to ROS 2 standards
- **Meshes**: STL/DAE files used for realistic visualization
- **Coordinate frames**: Follow ROS TF conventions

The robot description supports:

- Accurate visualization in RViz
- Physics-based simulation in Gazebo
- Integration with controllers, sensors, and navigation stacks

---

## Dependencies

Make sure the following are installed:

```bash
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros-pkgs
```

---

## How to Build

```bash
cd ~/ros_robots/skratch_ws
colcon build --symlink-install --packages-select skratch_description
source install/setup.bash
```

---

## Visualize the Robot in RViz

```bash
ros2 launch skratch_description description.launch.py
```

This will:
- Load the URDF model
- Publish TF frames
- Display the robot model in RViz

---

## Use in Gazebo Simulation

The robot description is automatically loaded when launching the Gazebo simulation from the `skratch_simulation` package:

```bash
ros2 launch skratch_gazebo gazebo.launch.py
```

---

## Package Purpose

This package is responsible only for the robot's description, including:

- Physical structure (base, wheels, sensor mounts)
- Visual appearance (meshes and colors)
- Joint definitions and kinematic tree
- Gazebo plugins configuration

**Note**: Controllers, sensors, and navigation behaviors are handled in separate packages (`skratch_navigation`, `skratch_localization`, etc.).

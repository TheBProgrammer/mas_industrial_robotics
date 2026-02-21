# skratch_localization

## Overview

The **`skratch_localization`** package provides localization functionality for the Skratch robot using **Nav2** in ROS 2.  
It launches the Nav2 localization stack (AMCL) with configurable parameters, map files, namespaces, and simulation time support.

This package is intended to be used together with mapping and navigation packages.

---

## Features

- Launches Nav2 localization stack
- Supports robot namespaces
- Configurable map file loading
- Supports simulation (`use_sim_time`)
- YAML-based localization parameter configuration

---

## Package Structure

```text
skratch_localization/
├── config/
│   └── localization.yaml        # Localization parameters
├── launch/
│   └── localization.launch.py   # Localization launch file
└── README.md
```

## Launch File
localization.launch.py
This launch file:

Loads a predefined or user-specified map

Starts the Nav2 localization nodes

Applies an optional robot namespace

Uses parameters from localization.yaml

It internally includes:

```bash

nav2_bringup/launch/localization_launch.py
```

## Launch Arguments
| Argument       | Default             | Description                    |
| -------------- | ------------------- | ------------------------------ |
| `use_sim_time` | `false`             | Use simulation time (Gazebo)   |
| `namespace`    | `""`                | Robot namespace                |
| `map_name`     | `rc_arena_sim`      | Map subdirectory name          |
| `map`          | `rc_arena_sim/map.yaml` | Full path to the map YAML file |
| `params`       | `localization.yaml` | Localization parameters file   |

> [!TIP]
> Maps are located in the `skratch_simulation/skratch_mapping/maps/` directory.
> Available maps: `rc_arena_sim`, `eval_arena`


Usage
Basic Launch
```bash

ros2 launch skratch_localization localization.launch.py
```

## Launch with Simulation Time
``` bash

ros2 launch skratch_localization localization.launch.py use_sim_time:=true
```
## Launch with Namespace
```bash

ros2 launch skratch_localization localization.launch.py namespace:=robot1
```
## Launch with Custom Map
```bash
ros2 launch skratch_localization localization.launch.py \
map:=/absolute/path/to/map.yaml
```
## Launch with Different Map Name
```bash
ros2 launch skratch_localization localization.launch.py \
map_name:=eval_arena
```
## Launch with Custom Parameters
```bash

ros2 launch skratch_localization localization.launch.py \
params:=/absolute/path/to/localization.yaml
```
## Dependencies
This package depends on the following ROS 2 packages:

nav2_bringup

launch

launch_ros

ament_index_python

Make sure Nav2 is installed:

```bash

sudo apt install ros-${ROS_DISTRO}-nav2-bringup
```
## Notes
The map YAML file must exist and be readable.
Localization parameters must be compatible with Nav2.
When using namespaces, all related nodes should use the same namespace.


# Skratch Navigation Package

## Overview
This package contains navigation configurations and performance evaluation tools for the Skratch robot using ROS2 Nav2 stack.

---

## Performance Analysis Summary

### Tested Configurations
Based on data collected from **92 navigation attempts** across different controller and planner combinations:

| Configuration | Controller | Planner | Test Runs |
|--------------|------------|---------|-----------|
| **Config 1** | MPPI | SmacHybrid | 40 |
| **Config 2** | MPPI | Smac2D | 25 |
| **Config 3** | MPPI | ThetaStar | 22 |
| **Config 4** | DWB | SmacHybrid | 5 |

---

### Analysis Results

#### 1. **Success Rate**

| Configuration | Success Rate | Rejected | Aborted |
|--------------|--------------|----------|---------|
| **MPPI + ThetaStar** | **95.5%** (21/22) | 4.5% | 0% |
| **MPPI + SmacHybrid** | **82.5%** (33/40) | 15% | 7.5% |
| **MPPI + Smac2D** | **76%** (19/25) | 12% | 12% |
| **DWB + SmacHybrid** | **100%** (5/5) | 0% | 0% |

> **Note**: DWB testing was limited (5 runs only), so statistical significance is lower.

#### 2. **Execution Time Performance**

Average total navigation time per goal (successful runs only):

| Configuration | Avg Time (s) | Best Time (s) | Worst Time (s) |
|--------------|--------------|---------------|----------------|
| **MPPI + ThetaStar** | **112.6s** | 1.7s | 188.0s |
| **MPPI + SmacHybrid** | 125.4s | 4.3s | 220.4s |
| **DWB + SmacHybrid** | 128.2s | 0.5s | 241.1s |
| **MPPI + Smac2D** | 155.3s | 11.9s | 494.5s |

#### 3. **Path Quality Metrics**

| Configuration | Avg Path Length (m) | Avg Smoothness (rad) | Avg Linear Vel (m/s) |
|--------------|---------------------|----------------------|----------------------|
| **MPPI + ThetaStar** | 13.67 | **0.36** (smoothest) | **0.418** (fastest) |
| **MPPI + SmacHybrid** | 13.75 | 3.69 | 0.342 |
| **DWB + SmacHybrid** | 15.52 | 4.46 | 0.262 |
| **MPPI + Smac2D** | 14.35 | 0.44 | 0.366 |

#### 4. **Safety Metrics**

Average minimum obstacle distance (higher is safer):

| Configuration | Min Obstacle Dist (m) | Near Collisions |
|--------------|----------------------|-----------------|
| **MPPI + SmacHybrid** | 0.477m | 80 total |
| **MPPI + ThetaStar** | **0.395m** | 100 total |
| **MPPI + Smac2D** | 0.498m | 1,508 total |
| **DWB + SmacHybrid** | 0.441m | 0 total |

> **Warning**: MPPI + Smac2D had significantly more near-collision events (1,508 vs <100 for others).

#### 5. **Resource Usage**

| Configuration | Avg CPU (%) | Avg RAM (MB) |
|--------------|-------------|--------------|
| **MPPI + ThetaStar** | **23.0%** | 3,721 |
| **MPPI + SmacHybrid** | 24.1% | 3,825 |
| **MPPI + Smac2D** | 24.5% | 3,828 |
| **DWB + SmacHybrid** | 24.6% | 3,869 |

---

## Chosen Configuration

### **MPPI Controller + ThetaStar Planner**

**Reasoning:**
1. **Highest Success Rate**: 95.5% (best reliability)
2. **Fastest Execution**: 112.6s average (11% faster than next best)
3. **Smoothest Paths**: Lowest path smoothness variation (0.36 rad)
4. **Best Velocity**: Highest average linear velocity (0.418 m/s)
5. **Lowest CPU Usage**: 23.0% average
6. **Zero Aborts**: All failures were rejections (safer failure mode)
7. **Tighter Spaces**: Best Performance in Tight Spaces

**Trade-offs:**
- Slightly more near-collision events than SmacHybrid (100 vs 80 total)
- Minimum obstacle distance is adequate but not the highest (0.395m)

**When to use alternatives:**
- **MPPI + SmacHybrid**: If maximum safety margin is critical (better obstacle clearance)

---

## Configuration Files

The package includes several parameter configurations:

| File | Controller | Planner | Use Case |
|------|------------|---------|----------|
| `nav2_params_mppi_theta.yaml` | MPPI | ThetaStar | **Recommended** (best performance) |
| `nav2_params_mppi_smac_hybrid.yaml` | MPPI | SmacHybrid | Alternative (better safety margins) |
| `nav2_params_mppi_smac_2d.yaml` | MPPI | Smac2D | Holonomic planning |
| `nav2_params_dwb_smac_hybrid.yaml` | DWB | SmacHybrid | DWB comparison baseline |
| `nav2_params_dwb_theta.yaml` | DWB | ThetaStar | DWB with ThetaStar planner |

---

## How to Run

### Prerequisites
```bash
# Source your workspace
cd ~/ros_robots/skratch_ws
source install/setup.bash
```

### Option 1: Run Nav2 Only (Manual Navigation)

Launch Nav2 navigation stack without automated metrics logging:

```bash
ros2 launch skratch_navigation nav2.launch.py
```

**With custom parameters:**
```bash
ros2 launch skratch_navigation nav2.launch.py params_file:=/path/to/custom_params.yaml
```

**Available parameters:**
- `params_file`: Path to Nav2 parameter file (default: `nav2_params_mppi_theta.yaml`)
- `use_sim_time`: Use simulation time - true/false (default: false)
- `namespace`: Robot namespace (default: empty)

**What it does:**
- Launches Nav2 navigation stack
- Allows manual goal setting via RViz or command line
- No automatic metrics logging

---

### Option 2: Run Nav2 with Metrics Logger (Automated Testing)

Launch Nav2 with automated sequential navigation and performance metrics collection:

```bash
ros2 launch skratch_navigation nav2_with_metrics.launch.py
```

**With custom configuration:**
```bash
ros2 launch skratch_navigation nav2_with_metrics.launch.py \
    params_file:=/path/to/params.yaml \
    goals_config:=/path/to/goals.yaml
```

**Available parameters:**
- `params_file`: Path to Nav2 parameter file (default: `nav2_params_mppi_theta.yaml`)
- `goals_config`: Path to goals configuration (default: `goals_config.yaml`)
- `use_sim_time`: Use simulation time - true/false (default: false)
- `namespace`: Robot namespace (default: empty)

**What it does:**
- Launches Nav2 navigation stack
- Starts `nav_metrics_logger` node
- Automatically sends goals sequentially from `goals_config.yaml`
- Logs navigation metrics to CSV file: `eval/nav_metrics_sequential.csv`
- Waits configured delay between goals (default: 3 seconds)

---

### Configuring Test Goals

Edit `config/goals_config.yaml` to define test waypoints:

```yaml
nav_metrics_logger:
  ros__parameters:
    goals:
      - {x: -8.707, y: -8.681, yaw: 1.599}
      - {x: -0.999, y: -0.220, yaw: 1.585}
      - {x: -0.062, y: 9.145, yaw: 0.001}
      # Add more goals...
    
    controller_name: 'MPPI'
    planner_name: 'ThetaStar'
    delay_between_goals: 3.0
```

---

## Pose Management Tools

This package includes tools to interactively save robot poses and navigate to them.

### 1. Save Poses
Interactive tool to save the current robot pose to `config/navigation_goals.yaml`.

```bash
# Standard Usage (Real Robot)
ros2 run skratch_navigation save_poses

# Simulation Usage (Gazebo)
ros2 run skratch_navigation save_poses --ros-args -p use_sim_time:=true
```

**Features:**
- Saves poses as `[x, y, yaw]`
- Supports custom frames (default: `base_link` relative to `map`)
- Robust handling of simulation time

### 2. Navigate to Pose
Send the robot to a saved named pose.

```bash
# Usage: ros2 run skratch_navigation navigate <POSE_NAME>

# Example (Simulation)
ros2 run skratch_navigation navigate WS01 --ros-args -p use_sim_time:=true
```

**Features:**
- Prints real-time distance and heading feedback
- Validates goal connectivity before sending

### 3. Additional Utility Scripts

#### Standalone Metrics Logger
Run the navigation metrics logger as a standalone node when you want to collect metrics without launching the full Nav2 stack via the metrics launch file.

```bash
ros2 run skratch_navigation nav_metrics_logger_standalone
```

### Navigation Goals Configuration

Saved poses are stored in `config/navigation_goals.yaml` in the following format:

```yaml
navigation_goals:
  POSE_NAME:
    - x_position
    - y_position
    - yaw_orientation
```

**Example:**
```yaml
navigation_goals:
  WS01:
    - -8.707
    - -8.681
    - 1.599
  WS02:
    - -0.999
    - -0.220
    - 1.585
```

---

## Collected Metrics

The metrics logger collects the following data for each navigation attempt:

### Temporal Metrics
- Total navigation time
- Planning time

### Path Quality
- Path length
- Path smoothness (curvature)

### Velocity Metrics
- Average/Max linear velocity
- Average/Max angular velocity
- Average/Max lateral velocity

### Dynamics
- Maximum linear acceleration
- Maximum linear jerk

### Safety
- Minimum obstacle distance
- Near-collision count

### System Resources
- Average CPU usage
- Maximum RAM usage
- Recovery behavior count

### Outcome
- SUCCESS / ABORTED / REJECTED

---

## Data Analysis

Navigation metrics are saved to: `eval/nav_metrics_sequential.csv`

You can analyze the data using:
- Python pandas/matplotlib
- Excel/LibreOffice Calc
- Custom analysis scripts

---

## Package Structure

```
skratch_navigation/
├── config/
│   ├── nav2_params_mppi_theta.yaml          # Recommended config
│   ├── nav2_params_mppi_smac_hybrid.yaml
│   ├── nav2_params_mppi_smac_2d.yaml
│   ├── nav2_params_dwb_smac_hybrid.yaml
│   └── goals_config.yaml                     # Test waypoints
├── launch/
│   ├── nav2.launch.py                        # Basic Nav2 launch
│   └── nav2_with_metrics.launch.py           # Nav2 + metrics logging
├── eval/
│   └── nav_metrics_sequential.csv            # Performance data
└── src/
    └── nav_metrics_logger.cpp                # Metrics collection node
```

---

## Tips for Navigation Tuning

1. **Adjust planner aggressiveness**: Modify `cost_penalty` in planner config
2. **Change safety margins**: Adjust `inflation_radius` in costmap config
3. **Tune controller responsiveness**: Modify MPPI `time_steps` and `batch_size`
4. **Path smoothness**: Adjust `w_smooth` and `w_data` in smoother config (SmacHybrid only)

---

## Contributing

When testing new configurations:
1. Update `goals_config.yaml` with controller/planner names
2. Run `nav2_with_metrics.launch.py`
3. Document results in this README
4. Compare metrics with baseline configurations

---

